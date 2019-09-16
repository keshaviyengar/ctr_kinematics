//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>
#include <include/ctr_kinematics/section_i.h>

CTRKinematics::CTRKinematics(ros::NodeHandle nh): nh_(nh){
    tip_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("tip_pose", 10);
    joint_sub_ = nh_.subscribe("joint_state", 10, &CTRKinematics::jointStateCallback, this);
}

CTRKinematics::~CTRKinematics(){}

void CTRKinematics::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(),1);
    c_joint_values(0) = msg->position.at(0);
    c_joint_values(1) = msg->position.at(1);
    c_joint_values(2) = msg->position.at(2);
    c_joint_values(3) = msg->position.at(3);
    c_joint_values(4) = msg->position.at(4);
    c_joint_values(5) = msg->position.at(5);

    Erl::debug_cout_g()<<"Joint values: "<<c_joint_values.transpose()<<std::endl;
    Robot_t::Transform fk = c_robot_.calcKinematic(c_joint_values, c_sample_);

    geometry_msgs::Pose tip_pose;
    tip_pose.position.x = fk.getX();
    tip_pose.position.y = fk.getY();
    tip_pose.position.z = fk.getZ();
    tip_pose.orientation.x = fk.getQuaternion().x();
    tip_pose.orientation.y = fk.getQuaternion().y();
    tip_pose.orientation.z = fk.getQuaternion().z();
    tip_pose.orientation.w = fk.getQuaternion().w();

    publishTipPose(tip_pose);
}

void CTRKinematics::publishTipPose(geometry_msgs::Pose &pose)
{
    tip_pose_pub_.publish(pose);
}

void CTRKinematics::run()
{

    size_t c_NSection = c_robot_.getNSections();
    size_t c_NTubes = c_robot_.getNTubes();
    ROS_INFO("General Stats");
    ROS_INFO_STREAM("Num sections: " << c_NSection);
    ROS_INFO_STREAM("Num tubes: " << c_NTubes);

    const ::CTR::types::SecVC<Real> *c_SecVC;
    const ::CTR::types::SecCC<Real> *c_SecCC;
    for(size_t iS(0);iS<c_NSection;iS++)
    {
        c_robot_.getSection(iS,&c_SecCC);
        c_robot_.getSection(iS,&c_SecVC);
        ERL_ASSERT( ((c_SecVC!=nullptr) || (c_SecCC!=nullptr)) );
        if(c_SecCC!=nullptr)
        {
            ROS_INFO("Constant Curvature");
            ROS_INFO_STREAM("section length: " << c_SecCC->getLength());
            ROS_INFO_STREAM("n tube: " << c_SecCC->ntube);
            ROS_INFO_STREAM("phi: " << c_SecCC->getPhi());
        }
        else
        {
            ROS_INFO("Variable Curvature");
            ROS_INFO_STREAM("section length: " << c_SecVC->getLength());
            ROS_INFO_STREAM("n tube: " << c_SecVC->ntube);
            ROS_INFO_STREAM("n phi: " << c_SecVC->getPhi());
        }
    }
}
