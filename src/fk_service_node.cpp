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
std::string c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_2_tube.xml";
Robot_t c_robot_ = Robot_t(c_sample_, c_xml_filename_); // Number of samples along robot shape and xml file location

void joint_state_to_vector(sensor_msgs::JointState &joint_state, Robot_t::VectorJ &joint_vector)
{
    std::vector<double> alpha = joint_state.position;
    std::vector<double> beta = joint_state.position;

    // Split into seperate vectors of alpha and beta
    std::stable_partition(begin(alpha), end(alpha), [](int i){return !(i % 2);});
    std::stable_partition(begin(beta), end(beta), [](int i){return (i % 2);});

    size_t num_sections = c_robot_.getNSections();

    size_t num_tubes = c_robot_.getNTubes();

    // First index always base rotation
    double rigid_theta = 4;
    joint_vector[0] = rigid_theta;

    // Iterate through variable section
    int beta_i = 0;
    int alpha_i = 0;
    int joint_vector_i = 1;
    for (int i=0; i!=(num_tubes - num_sections); ++i)
    {
        double alpha_1 = alpha[alpha_i];
        double alpha_2 = alpha[alpha_i + 1];
        double phi = beta[beta_i];
        joint_vector[joint_vector_i] = phi;
        joint_vector_i += 1;
        joint_vector[joint_vector_i] = alpha_1;
        joint_vector_i += 1;
        joint_vector[joint_vector_i] = alpha_2;
        joint_vector_i += 1;
        beta_i += 1;
        alpha_i += 1;
    }

    alpha_i = num_tubes - num_sections;
    // Iterate through fixed sections
    for (int i=0; i!=(2 * num_sections - num_tubes); ++i)
    {
        double alpha_1 = alpha[alpha_i];
        double phi = beta[beta_i];
        joint_vector[joint_vector_i] = phi;
        joint_vector_i += 1;
        joint_vector[joint_vector_i] = alpha_1;
        joint_vector_i += 1;
        beta_i += 1;
        alpha_i += 1;
    }
}

bool fk_service_callback(ctr_kinematics::ComputeForwardKinematics::Request& request,
                         ctr_kinematics::ComputeForwardKinematics::Response& response)
{
    Robot_t::VectorJ joint_vector(c_robot_.getNJointValues(), 1);
    joint_state_to_vector(request.joint_values, joint_vector);
    ROS_INFO_STREAM("Joint State\n" << request.joint_values);
    ROS_INFO_STREAM("Joint Vector\n" << joint_vector);

    response.end_effector_pose.position.x = 0;
    response.end_effector_pose.position.y = 0;
    response.end_effector_pose.position.z = 0;

    response.end_effector_pose.orientation.x = 0;
    response.end_effector_pose.orientation.y = 0;
    response.end_effector_pose.orientation.z = 0;
    response.end_effector_pose.orientation.w = 0;
    return true;
    /*
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
     */
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

