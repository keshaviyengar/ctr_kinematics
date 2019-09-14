// This file is part of CTR, a kinematics library for concentric tube robots
//
// Copyright (C) 2017 Konrad Leibrandt <konrad.lei@gmx.de>
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <Erl/Utility/debug.h>
#include <ros/ros.h>
#include <ctr_kinematics/ctr_robot.h>
#include <ctr_kinematics/ctr_kinematics.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

typedef double Real;
typedef CTR::Robot<Real> Robot_t;

// Forward kinematics subscribes to joint state and publishes a pose message (Point position, Quaternion orientation)
void forwardKinematicsCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    size_t      c_sample       = 256;
    std::string c_xml_filename = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    Robot_t c_robot = Robot_t(c_sample, c_xml_filename);
    Robot_t::VectorJ c_joint_values(c_robot.getNJointValues(),1);
    c_joint_values(0) = msg->position.at(0);
    c_joint_values(1) = msg->position.at(1);
    c_joint_values(2) = msg->position.at(2);
    c_joint_values(3) = msg->position.at(3);
    c_joint_values(4) = msg->position.at(4);
    c_joint_values(5) = msg->position.at(5);

    Erl::debug_cout_g()<<"Joint values: "<<c_joint_values.transpose()<<std::endl;
    Robot_t::Transform fk = c_robot.calcKinematic(c_joint_values, c_sample);

    geometry_msgs::Pose tip_pose;
    tip_pose.position.x = fk.getX();
    tip_pose.position.y = fk.getY();
    tip_pose.position.z = fk.getZ();
    tip_pose.orientation.x = fk.getQuaternion().x();
    tip_pose.orientation.y = fk.getQuaternion().y();
    tip_pose.orientation.z = fk.getQuaternion().z();
    tip_pose.orientation.w = fk.getQuaternion().w();

    ROS_INFO_STREAM("Tip pose, x: " << tip_pose.position.x << " y: " << tip_pose.position.y << " z: " << tip_pose.position.z);

}

int main(int _argc, char *_argv[])
{
    ros::init(_argc, _argv, "ctr_kinematics_node");
    ros::NodeHandle n;

    CTRKinematics ctr_kinematics(n);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
