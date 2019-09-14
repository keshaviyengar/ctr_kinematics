//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>

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