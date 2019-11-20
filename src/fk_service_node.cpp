#include <Erl/Utility/debug.h>
#include <ros/ros.h>
#include <ctr_kinematics/ctr_robot.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <ctr_kinematics/ComputeForwardKinematics.h>


typedef double Real;
typedef CTR::Robot<Real> Robot_t;

// Initialize robot kinematics
size_t c_sample_ = 256;
std::string c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
Robot_t c_robot_ = Robot_t(c_sample_, c_xml_filename_); // Number of samples along robot shape and xml file location


bool fk_service_callback(ctr_kinematics::ComputeForwardKinematics::Request& request,
                         ctr_kinematics::ComputeForwardKinematics::Response& response)
{
    // convert sensor_msgs JointState to VectorJ joints
    // For the 2 section 3 tube robot there will be 6 joint values
    // [alpha_rigid, phi_0 = phi_1, alpha_0, alpha_1, ]

    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(), 1);
    request.joint_values.position.resize(c_robot_.getNJointValues());


    c_joint_values(0) = request.joint_values.position.at(0);
    c_joint_values(1) = request.joint_values.position.at(1);
    c_joint_values(2) = request.joint_values.position.at(2);
    c_joint_values(3) = request.joint_values.position.at(3);
    c_joint_values(4) = request.joint_values.position.at(4);
    c_joint_values(5) = request.joint_values.position.at(5);

    // Compute FK with Robot_t
    Robot_t::Transform fk = c_robot_.calcKinematic(c_joint_values, c_sample_);

    response.end_effector_pose.position.x = fk.getX();
    response.end_effector_pose.position.y = fk.getY();
    response.end_effector_pose.position.z = fk.getZ();
    response.end_effector_pose.orientation.x = fk.getQuaternion().normalized().x();
    response.end_effector_pose.orientation.y = fk.getQuaternion().normalized().y();
    response.end_effector_pose.orientation.z = fk.getQuaternion().normalized().z();
    response.end_effector_pose.orientation.w = fk.getQuaternion().normalized().w();

    return true;
}

int main(int _argc, char *_argv[])
{
    ros::init(_argc, _argv, "ctr_fk_service_node");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    // every few seconds sample a new joint and publish

    ros::ServiceServer fk_service = n.advertiseService("ctr_fk_service", fk_service_callback);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

