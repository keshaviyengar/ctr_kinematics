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
#include <geometry_msgs/PoseStamped.h>

typedef double Real;
typedef CTR::Robot<Real> Robot_t;

class CTRKinematics
{
public:
    CTRKinematics(ros::NodeHandle nh);
    ~CTRKinematics();

    void publishConfiguration();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void DesiredTipPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void run();

    // Publish sampled joints for now
    void sampleJointSpace(Robot_t::VectorJ& joint_values, sensor_msgs::JointState& joint_state);
    void publishASampledJointAndTipPose(const ros::TimerEvent&);

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    ros::Subscriber tip_pose_sub_;
    ros::Publisher tip_pose_pub_;

    ros::Publisher sampled_joint_pub_;
    ros::Publisher sampled_desired_tip_pose_pub_;
    ros::Publisher desired_tip_pose_viz_pub_;
    ros::Publisher current_tip_pose_viz_pub_;
    ros::Publisher tip_pose_estimate_viz_pub_;

    ros::Publisher tip_error_pub_;
    ros::Publisher joint_error_pub_;

    // Timers for sampling for testing
    ros::Timer new_sample_;

    size_t c_sample_;
    std::string c_xml_filename_;
    Robot_t c_robot_;

    CT_RNG c_rng_; CT_RND<Real> c_rnd_;

    Robot_t::VectorJ dof_index_;

    // Keep track of goals
    Robot_t::Transform desired_tip_pose_;
    Robot_t::Transform current_tip_pose_;
    Robot_t::VectorJ current_joints_;
    Robot_t::VectorJ desired_joints_;
    Robot_t::Vector6 jacobian_tip_estimate_;

};


#endif //CTR_KINEMATICS_CTR_KINEMATICS_H
