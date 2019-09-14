//
// Created by keshav on 13/09/19.
//

#ifndef CTR_KINEMATICS_CTR_KINEMATICS_H
#define CTR_KINEMATICS_KINEMATICS_H

#include <Erl/Utility/debug.h>

#include <ros/ros.h>
#include <ctr_kinematics/ctr_robot.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

typedef double Real;
typedef CTR::Robot<Real> Robot_t;

class CTRKinematics
{
public:
    CTRKinematics(ros::NodeHandle nh);
    ~CTRKinematics();

    void publishTipPose(geometry_msgs::Pose &pose);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    ros::Publisher tip_pose_pub_;

    size_t c_sample_ = 256;
    std::string c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    Robot_t c_robot_ = Robot_t(c_sample_, c_xml_filename_);

};


#endif //CTR_KINEMATICS_CTR_KINEMATICS_H
