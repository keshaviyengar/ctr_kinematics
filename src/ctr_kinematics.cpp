//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>
#include <tf/transform_broadcaster.h>

CTRKinematics::CTRKinematics(ros::NodeHandle nh): nh_(nh){
    c_sample_ = 256;
    c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    c_robot_ = Robot_t(c_sample_, c_xml_filename_);

    tip_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("tip_pose", 10);
    configuration_pub_ = nh_.advertise<ctm2_visualization::Visualizer>("configuration_state", 10);
    joint_sub_ = nh_.subscribe("joint_state", 10, &CTRKinematics::jointStateCallback, this);

    sampled_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
    new_joints_ = nh_.createTimer(ros::Duration(5.0), &CTRKinematics::publishASampledJoints, this);
}

CTRKinematics::~CTRKinematics(){}

void CTRKinematics::publishASampledJoints(const ros::TimerEvent&)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(),1);
    c_robot_.getRandomJointValues(c_rng_,c_rnd_,c_joint_values);

    sensor_msgs::JointState joint_state;
    joint_state.position.resize(6);
    joint_state.position.at(0) = c_joint_values(0);
    joint_state.position.at(1) = c_joint_values(1);
    joint_state.position.at(2) = c_joint_values(2);
    joint_state.position.at(3) = c_joint_values(3);
    joint_state.position.at(4) = c_joint_values(4);
    joint_state.position.at(5) = c_joint_values(5);

    sampled_joint_pub_.publish(joint_state);
}

void CTRKinematics::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(),1);
    c_joint_values(0) = msg->position.at(0);
    c_joint_values(1) = msg->position.at(1);
    c_joint_values(2) = msg->position.at(2);
    c_joint_values(3) = msg->position.at(3);
    c_joint_values(4) = msg->position.at(4);
    c_joint_values(5) = msg->position.at(5);

    Robot_t::Transform fk = c_robot_.calcKinematic(c_joint_values, c_sample_);

    geometry_msgs::Pose tip_pose;
    tip_pose.position.x = fk.getX();
    tip_pose.position.y = fk.getY();
    tip_pose.position.z = fk.getZ();
    tip_pose.orientation.x = fk.getQuaternion().x();
    tip_pose.orientation.y = fk.getQuaternion().y();
    tip_pose.orientation.z = fk.getQuaternion().z();
    tip_pose.orientation.w = fk.getQuaternion().w();

    tip_pose_pub_.publish(tip_pose);

    publishConfiguration();
}

void CTRKinematics::publishConfiguration()
{
    ctm2_visualization::Visualizer config;
    size_t c_NSection = c_robot_.getNSections();
    size_t c_NTubes = c_robot_.getNTubes();

    const ::CTR::types::SecVC<Real> *c_SecVC;
    const ::CTR::types::SecCC<Real> *c_SecCC;
    for(size_t iS(0);iS<c_NSection;iS++)
    {
        c_robot_.getSection(iS,&c_SecCC);
        c_robot_.getSection(iS,&c_SecVC);
        ERL_ASSERT( ((c_SecVC!=nullptr) || (c_SecCC!=nullptr)) );
        config.header.stamp = ros::Time::now();
        config.segment_id = iS;
        if(c_SecCC!=nullptr)
        {
            config.phi = c_SecCC->getTubeAlphaTip(c_SecCC->ntube - 1);
            config.length = c_SecCC->getPhi();
            config.kappa = c_SecCC->getCurvature(c_sample_);
            ROS_INFO_STREAM("Section Info: " << *c_SecCC);
        }
        else
        {
            config.phi = c_SecVC->getTubeAlphaTip(c_SecVC->ntube - 1);
            config.length = c_SecVC->getPhi();
            config.kappa = c_SecVC->getCurvature(c_sample_);
            ROS_INFO_STREAM("Section Info: " << *c_SecVC);
        }
        configuration_pub_.publish(config);
    }

    // Publish sampled transforms
    static tf::TransformBroadcaster br;
    size_t c_NSamples = c_robot_.getMaxSamples();
    for(size_t iSp(0);iSp<c_NSamples;iSp++)
    {
        tf::Transform transform;
        Robot_t::Transform transform_sample = c_robot_.getTransformSample(iSp);

        tf::Vector3 translation(transform_sample.getX(), transform_sample.getY(), transform_sample.getZ());
        transform.setOrigin(translation);
        tf::Quaternion rotation;
        rotation.setX(transform_sample.getQuaternion().x());
        rotation.setY(transform_sample.getQuaternion().y());
        rotation.setZ(transform_sample.getQuaternion().z());
        rotation.setW(transform_sample.getQuaternion().w());

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world",std::to_string(iSp)));
    }
}
