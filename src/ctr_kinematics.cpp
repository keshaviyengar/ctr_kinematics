//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/Float32.h>

CTRKinematics::CTRKinematics(ros::NodeHandle nh): nh_(nh){
    c_sample_ = 256;
    c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    c_robot_ = Robot_t(c_sample_, c_xml_filename_); // Number of samples along robot shape and xml file locaion

    tip_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("tip_pose", 10);

    //joint_sub_ = nh_.subscribe("joint_state", 10, &CTRKinematics::jointStateCallback, this);
    tip_pose_sub_ = nh_.subscribe("desired_tip_pose", 10, &CTRKinematics::DesiredTipPoseCallback, this);

    sampled_desired_tip_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("desired_tip_pose", 10);
    // Visualize poses
    desired_tip_pose_viz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("viz_desired_tip_pose", 10);
    current_tip_pose_viz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("viz_current_tip_pose", 10);
    tip_pose_estimate_viz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("viz_jacobian_tip_pose", 10);

    // Publish errors
    tip_error_pub_ = nh_.advertise<std_msgs::Float32>("tip_error", 10);
    joint_error_pub_ = nh_.advertise<std_msgs::Float32>("joint_error", 10);

    // Sampling timers for testing
    new_sample_ = nh_.createTimer(ros::Duration(10.0), &CTRKinematics::publishASampledJointAndTipPose, this);

    // Initialize joint values and desired tip pose
    current_joints_.resize(c_robot_.getNJointValues(), 1);
    desired_joints_.resize(c_robot_.getNJointValues(), 1);
    c_robot_.getRandomJointValues(c_rng_, c_rnd_, current_joints_);
    desired_joints_ = current_joints_;

    current_tip_pose_  = c_robot_.calcKinematic(current_joints_, c_sample_);

    // Set first sampled desired pose to be the same as the achieved tip pose, to see how an error of zero looks like
    desired_tip_pose_ = current_tip_pose_;
}

CTRKinematics::~CTRKinematics(){}

// Sample random joints and return a sensor_msgs::JointState
void CTRKinematics::sampleJointSpace(Robot_t::VectorJ &joint_values, sensor_msgs::JointState &joint_state)
{
    c_robot_.getRandomJointValues(c_rng_, c_rnd_, joint_values);

    joint_state.position.resize(6);
    joint_state.position.at(0) = joint_values(0);
    joint_state.position.at(1) = joint_values(1);
    joint_state.position.at(2) = joint_values(2);
    joint_state.position.at(3) = joint_values(3);
    joint_state.position.at(4) = joint_values(4);
    joint_state.position.at(5) = joint_values(5);
}

// Sample a JoinState then publish the desired tip pose
void CTRKinematics::publishASampledJointAndTipPose(const ros::TimerEvent&)
{
    // Set desired tip pose to a random tip pose
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(),1);
    sensor_msgs::JointState joint_state;

    sampleJointSpace(c_joint_values, joint_state);

    desired_tip_pose_ = c_robot_.calcKinematic(c_joint_values, c_sample_);
    desired_tip_pose_.getQuaternion().normalized();
    desired_joints_ = c_joint_values;

    //Set a random joint state
    sampleJointSpace(current_joints_, joint_state);
    current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_);
}

// Used to explore joint space by receiving a joint state and publishing the computed end effector
void CTRKinematics::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    Robot_t::VectorJ c_joint_values(c_robot_.getNJointValues(), 1);
    c_joint_values(0) = 0;
    c_joint_values(1) = fmin(fmax(0.0, msg->position.at(0)), 0.06647);
    c_joint_values(2) = msg->position.at(1);
    c_joint_values(3) = msg->position.at(3);
    c_joint_values(4) = fmin(fmax(0.0, msg->position.at(4)), 0.06647);
    c_joint_values(5) = msg->position.at(5);

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

// Receives a desired tip pose, sets a private variable desired_tip_pose_
void CTRKinematics::DesiredTipPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    // TF pose to eigen transform
    tf::Pose desired_tip_pose;
    tf::poseMsgToTF(msg->pose, desired_tip_pose);
    Eigen::Affine3d desired_tip_eigen;
    tf::poseTFToEigen(desired_tip_pose, desired_tip_eigen);
    Robot_t::Transform desired_tip_transform;
    desired_tip_pose_ = Robot_t::Transform::fromEigenMatrix(desired_tip_eigen.matrix());
}

// Main run function
void CTRKinematics::run()
{
    Real step_return;
    current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_, step_return);
    Robot_t::Transform delta_tip_pose = current_tip_pose_.inv() * desired_tip_pose_;
    Robot_t::Vector6 delta_tip_vector;

    // Get tip pose error as vector rather than transform
    delta_tip_vector.setZero();
    delta_tip_vector[0] = delta_tip_pose.getTranslation().x();
    delta_tip_vector[1] = delta_tip_pose.getTranslation().y();
    delta_tip_vector[2] = delta_tip_pose.getTranslation().z();

    // Publish the cartesian and joint space error
    std_msgs::Float32 tip_error;
    tip_error.data = delta_tip_pose.getTranslation().norm();
    tip_error_pub_.publish(tip_error);
    std_msgs::Float32 joint_error;
    tip_error.data = (desired_joints_ - current_joints_).norm();
    joint_error_pub_.publish(tip_error);

    // compute the jacobian
    double trans_diff = 0.06647 / 1000; // perturbation in translation joints
    double rot_diff = 2; // perturbation in rotation joint
    Robot_t::MatJacobian j_tip(6, 6);

    c_robot_.calcJacobian(current_joints_, step_return, trans_diff, rot_diff, j_tip);
    //c_robot_.calcJacobian(current_joints_, current_tip_pose_, c_sample_, trans_diff, rot_diff, j_tip);

    Robot_t::Vector6 j_computed_tip;
    // Calculate jacobian tip estimate
    jacobian_tip_estimate_ = j_tip * current_joints_;

    // Change in joints
    Robot_t::VectorJ delta_q;

    // transpose method
    delta_q = j_tip.transpose() * delta_tip_vector;

    // dls method
    // double lambda = 0.00;
    // Robot_t::MatJacobian j_damped(6,6);
    // j_damped = j_tip * j_tip.transpose() + pow(lambda, 2) * Robot_t::MatJacobian::Identity(6,6);
    // delta_q = j_tip.transpose() * j_damped.inverse() * (delta_tip_vector);

    // Limit velocity
    double ang_vel_limit = 0.5 / 2;
    double ext_vel_limit = 0.001 / 2;
    delta_q(0) = fmin(fmax(-ang_vel_limit, delta_q(0)), ang_vel_limit);
    delta_q(1) = fmin(fmax(-ext_vel_limit, delta_q(1)), ext_vel_limit);
    delta_q(2) = fmin(fmax(-ang_vel_limit, delta_q(2)), ang_vel_limit);
    delta_q(3) = fmin(fmax(-ang_vel_limit, delta_q(3)), ang_vel_limit);
    delta_q(4) = fmin(fmax(-ext_vel_limit, delta_q(4)), ext_vel_limit);
    delta_q(5) = fmin(fmax(-ang_vel_limit, delta_q(5)), ang_vel_limit);

    // set current q to current q + delta q
    current_joints_+= delta_q;

    // Limit joints
    current_joints_(1) = fmin(fmax(0.0, current_joints_(1)), 0.06647);
    current_joints_(4) = fmin(fmax(0.0, current_joints_(4)), 0.06647);

    current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_);

    // Publish new joints
    publishConfiguration();
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

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world", std::to_string(iSp)));
    }
    // Publish desired pose
    geometry_msgs::PoseStamped tip_pose;
    tip_pose.header.stamp = ros::Time::now();
    tip_pose.header.frame_id = "world";
    tip_pose.pose.position.x = desired_tip_pose_.getX() * 100;
    tip_pose.pose.position.y = desired_tip_pose_.getY() * 100;
    tip_pose.pose.position.z = desired_tip_pose_.getZ() * 100;
    tip_pose.pose.orientation.x = desired_tip_pose_.getQuaternion().normalized().x();
    tip_pose.pose.orientation.y = desired_tip_pose_.getQuaternion().normalized().y();
    tip_pose.pose.orientation.z = desired_tip_pose_.getQuaternion().normalized().z();
    tip_pose.pose.orientation.w = desired_tip_pose_.getQuaternion().normalized().w();
    desired_tip_pose_viz_pub_.publish(tip_pose);

    // Publish current pose
    tip_pose.header.stamp = ros::Time::now();
    tip_pose.header.frame_id = "world";
    tip_pose.pose.position.x = current_tip_pose_.getX() * 100;
    tip_pose.pose.position.y = current_tip_pose_.getY() * 100;
    tip_pose.pose.position.z = current_tip_pose_.getZ() * 100;
    tip_pose.pose.orientation.x = current_tip_pose_.getQuaternion().normalized().x();
    tip_pose.pose.orientation.y = current_tip_pose_.getQuaternion().normalized().y();
    tip_pose.pose.orientation.z = current_tip_pose_.getQuaternion().normalized().z();
    tip_pose.pose.orientation.w = current_tip_pose_.getQuaternion().normalized().w();
    current_tip_pose_viz_pub_.publish(tip_pose);

    // Publish jacobian estimated tip pose
    tip_pose.header.stamp = ros::Time::now();
    tip_pose.header.frame_id = "world";
    tip_pose.pose.position.x = jacobian_tip_estimate_.x() * 100;
    tip_pose.pose.position.y = jacobian_tip_estimate_.y() * 100;
    tip_pose.pose.position.z = jacobian_tip_estimate_.z() * 100;
    tf::Quaternion orientation;
    orientation.setEuler(jacobian_tip_estimate_[3], jacobian_tip_estimate_[4], jacobian_tip_estimate_[5]);
    tip_pose.pose.orientation.x = orientation.x();
    tip_pose.pose.orientation.y = orientation.y();
    tip_pose.pose.orientation.z = orientation.z();
    tip_pose.pose.orientation.w = orientation.w();
    tip_pose_estimate_viz_pub_.publish(tip_pose);
}
