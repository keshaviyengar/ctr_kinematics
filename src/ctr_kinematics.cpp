//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <tf_conversions/tf_eigen.h>

CTRKinematics::CTRKinematics(ros::NodeHandle nh): nh_(nh){
    c_sample_ = 256;
    c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    c_robot_ = Robot_t(c_sample_, c_xml_filename_);

    tip_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("tip_pose", 10);
    configuration_pub_ = nh_.advertise<ctm2_visualization::Visualizer>("configuration_state", 10);

    joint_sub_ = nh_.subscribe("joint_state", 10, &CTRKinematics::jointStateCallback, this);
    tip_pose_sub_ = nh_.subscribe("desired_tip_pose", 10, &CTRKinematics::DesiredTipPoseCallback, this);

    sampled_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("sampled_joint_state", 10);
    sampled_desired_tip_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("desired_tip_pose", 10);

    // Sampling timers for testing
    new_sample_ = nh_.createTimer(ros::Duration(5.0), &CTRKinematics::publishASampledJointAndTipPose, this);
}

CTRKinematics::~CTRKinematics(){}

void CTRKinematics::sampleJointSpace(Robot_t::VectorJ &joint_values, sensor_msgs::JointState &joint_state)
{
    c_robot_.getRandomJointValues(c_rng_,c_rnd_, joint_values);

    joint_state.position.resize(6);
    joint_state.position.at(0) = joint_values(0);
    joint_state.position.at(1) = joint_values(1);
    joint_state.position.at(2) = joint_values(2);
    joint_state.position.at(3) = joint_values(3);
    joint_state.position.at(4) = joint_values(4);
    joint_state.position.at(5) = joint_values(5);
}

void CTRKinematics::publishASampledJointAndTipPose(const ros::TimerEvent&)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(),1);
    sensor_msgs::JointState joint_state;

    sampleJointSpace(c_joint_values, joint_state);

    // Publish the sampled joint state
    sampled_joint_pub_.publish(joint_state);

    Robot_t::Transform fk = c_robot_.calcKinematic(c_joint_values, c_sample_);
    fk.getQuaternion().normalized();

    geometry_msgs::Pose tip_pose;
    tip_pose.position.x = fk.getX();
    tip_pose.position.y = fk.getY();
    tip_pose.position.z = fk.getZ();
    tip_pose.orientation.x = fk.getQuaternion().normalized().x();
    tip_pose.orientation.y = fk.getQuaternion().normalized().y();
    tip_pose.orientation.z = fk.getQuaternion().normalized().z();
    tip_pose.orientation.w = fk.getQuaternion().normalized().w();
    sampled_desired_tip_pose_pub_.publish(tip_pose);
}

void CTRKinematics::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(), 1);
    c_joint_values(0) = 0;
    c_joint_values(1) = fmin(fmax(0.0, msg->position.at(0)), 0.06647);
    c_joint_values(2) = msg->position.at(1);
    c_joint_values(3) = msg->position.at(3);
    c_joint_values(4) = fmin(fmax(0.0, msg->position.at(4)), 0.06647);
    c_joint_values(5) = msg->position.at(5);

    ROS_INFO_STREAM("Joint values: " << c_joint_values);

    Robot_t::Transform fk = c_robot_.calcKinematic(c_joint_values, c_sample_);
    fk.getQuaternion().normalized();

    geometry_msgs::Pose tip_pose;
    tip_pose.position.x = fk.getX();
    tip_pose.position.y = fk.getY();
    tip_pose.position.z = fk.getZ();
    tip_pose.orientation.x = fk.getQuaternion().normalized().x();
    tip_pose.orientation.y = fk.getQuaternion().normalized().y();
    tip_pose.orientation.z = fk.getQuaternion().normalized().z();
    tip_pose.orientation.w = fk.getQuaternion().normalized().w();

    tip_pose_pub_.publish(tip_pose);

    publishConfiguration();
}

void CTRKinematics::DesiredTipPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
    // TF pose to eigen transform
    tf::Pose desired_tip_pose;
    tf::poseMsgToTF(*msg, desired_tip_pose);
    Eigen::Affine3d desired_tip_eigen;
    tf::poseTFToEigen(desired_tip_pose, desired_tip_eigen);
    Robot_t::Transform desired_tip_transform;
    desired_tip_transform = Robot_t::Transform::fromEigenMatrix(desired_tip_eigen.matrix());

    desired_tip_pose_ = desired_tip_transform;

    Robot_t::MatJacobian j_tip(6,6);
    Robot_t::RReal finite_diff_trans = 0.001;
    Robot_t::RReal finite_diff_rot = 0.001;
    c_robot_.calcJacobian(c_robot_.getJointValues(),desired_tip_transform, c_sample_, finite_diff_trans, finite_diff_rot, j_tip);
    ROS_INFO_STREAM("Current joints: \n" << c_robot_.getJointValues());
    Robot_t::Vector6 del_tip;
    del_tip << desired_tip_transform.getX(), desired_tip_transform.getY(), desired_tip_transform.getZ(), 0, 0, 0;
    ROS_INFO_STREAM("delta tip:\n" << del_tip);
    ROS_INFO_STREAM("delta joints:\n" << j_tip.completeOrthogonalDecomposition().pseudoInverse() * del_tip);
}

void CTRKinematics::run()
{
    // check for current tip pose and desired tip pose
    Robot_t::VectorJ current_q(c_robot_.getNJointValues(), 1);
    current_q = c_robot_.getJointValues();
    Robot_t::Transform current_tip_pose = c_robot_.calcKinematic(current_q, c_sample_);

    // compute delta tip pose
    Robot_t::Transform delta_tip_pose = desired_tip_pose_.inv() * current_tip_pose;

    // compute the jacobian
    Robot_t::MatJacobian j_tip(6,6);
    c_robot_.calcJacobian(current_q, desired_tip_pose_, c_sample_, 0.001, 0.001, j_tip);

    // multiply jacobian and delta tip pose to get delta joints
    Robot_t::VectorJ delta_tip_vector;
    delta_tip_vector << delta_tip_pose.getX(), delta_tip_pose.getY(), delta_tip_pose.getZ(), 0, 0, 0;
    Robot_t::VectorJ delta_q;
    delta_q = j_tip.completeOrthogonalDecomposition().pseudoInverse() * delta_tip_vector;

    // set current q to current q + delta q
    current_q = current_q + delta_q;

    // Publish new joints
    ROS_INFO_STREAM("New joints: " << current_q);
}

// Possibly use this for updating joints with a run function in main for iterative solving
void CTRKinematics::publishConfiguration()
{
    // Publish sampled transforms
    static tf::TransformBroadcaster br;
    size_t c_NSamples = c_robot_.getMaxSamples();
    for(size_t iSp(0);iSp<c_NSamples;iSp++)
    {
        tf::Transform transform;
        Robot_t::Transform transform_sample = c_robot_.getTransformSample(iSp);

        tf::Vector3 translation(transform_sample.getX(), transform_sample.getY(), transform_sample.getZ());
        transform.setOrigin(translation * 100);
        tf::Quaternion rotation;
        rotation.setX(transform_sample.getQuaternion().x());
        rotation.setY(transform_sample.getQuaternion().y());
        rotation.setZ(transform_sample.getQuaternion().z());
        rotation.setW(transform_sample.getQuaternion().w());

        rotation.normalize();
        transform.setRotation(rotation);

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world",std::to_string(iSp)));
    }
}
