//
// Created by keshav on 13/09/19.
//

#ifndef CTR_KINEMATICS_CTR_KINEMATICS_H
#define CTR_KINEMATICS_KINEMATICS_H

#include <Erl/Utility/debug.h>
#include <include/ctr_kinematics/section_i.h>

#include <ros/ros.h>
#include <ctr_kinematics/ctr_robot.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <ctm2_visualization/Visualizer.h>

typedef double Real;
typedef CTR::Robot<Real> Robot_t;

class CTRKinematics
{
public:
    CTRKinematics(ros::NodeHandle nh);
    ~CTRKinematics();

    void publishConfiguration();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    // Publish sampled joints for now
    void publishASampledJoints(const ros::TimerEvent&);


private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    ros::Publisher tip_pose_pub_;
    ros::Publisher configuration_pub_;

    ros::Publisher sampled_joint_pub_;
    ros::Timer new_joints_;

    size_t c_sample_;
    std::string c_xml_filename_;
    Robot_t c_robot_;

    CT_RNG c_rng_; CT_RND<Real> c_rnd_;

};


#endif //CTR_KINEMATICS_CTR_KINEMATICS_H
