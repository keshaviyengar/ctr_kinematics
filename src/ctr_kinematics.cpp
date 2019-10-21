//
// Created by keshav on 13/09/19.
//

#include <ctr_kinematics/ctr_kinematics.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/Float32.h>
#include <angles/angles.h>

#include <random>
#include <cfloat>

CTRKinematics::CTRKinematics(ros::NodeHandle nh): nh_(nh){
    c_sample_ = 256;
    c_xml_filename_ = "/home/keshav/catkin_ws/src/ctr_kinematics/ctr_resources/ctr_sec2_tub3.xml";
    c_robot_ = Robot_t(c_sample_, c_xml_filename_); // Number of samples along robot shape and xml file location

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
    //new_sample_ = nh_.createTimer(ros::Duration(10.0), &CTRKinematics::publishCurrentJointAndDeltaTipPose, this);

    // Initialize joint values and desired tip pose
    current_joints_.resize(c_robot_.getNJointValues(), 1);
    desired_joints_.resize(c_robot_.getNJointValues(), 1);
    c_robot_.getRandomJointValues(c_rng_, c_rnd_, current_joints_);
    desired_joints_ = current_joints_;
    jacobian_tip_estimate_current_joints_ = current_joints_;

    // Reduce the degrees of freedom
    dof_index_.resize(c_robot_.getNJointValues(), 1);
    dof_index_[0] = 1; // rigid outer tube base angle
    dof_index_[1] = 1; // extension tube 1 = extension tube 2
    dof_index_[2] = 0; // tube 1 base angle
    dof_index_[3] = 0; // tube 2 base angle
    dof_index_[4] = 0; // tube 3 extension
    dof_index_[5] = 0; // tube 3 rotation

    // Add constant joint values even if dof is off
    constant_joint_values_.resize(c_robot_.getNJointValues(), 1);
    constant_joint_values_[0] = 0; // rigid outer tube base angle
    constant_joint_values_[1] = 0; //0.06647 / 2; // extension tube 1 = extension tube 2
    constant_joint_values_[2] = 0; // tube 1 base angle
    constant_joint_values_[3] = 0; // tube 2 base angle
    constant_joint_values_[4] = 0; // tube 3 extension
    constant_joint_values_[5] = 0; // tube 3 rotation

    current_tip_pose_  = c_robot_.calcKinematic(current_joints_, c_sample_);

    // Set first sampled desired pose to be the same as the achieved tip pose, to see how an error of zero looks like
    desired_tip_pose_ = current_tip_pose_;
    jacobian_tip_estimate_ = current_tip_pose_;
}

CTRKinematics::~CTRKinematics(){}

// Sample random joints and return a sensor_msgs::JointState
void CTRKinematics::sampleJointSpace(Robot_t::VectorJ &joint_values, sensor_msgs::JointState &joint_state)
{
    c_robot_.getRandomJointValues(c_rng_, c_rnd_, joint_values);

    joint_values = joint_values.cwiseProduct(dof_index_);
    joint_values += constant_joint_values_;

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
    desired_joints_ = c_joint_values;

    //Set a random joint state
    sampleJointSpace(current_joints_, joint_state);
    current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_);
    jacobian_tip_estimate_current_joints_ = current_joints_;
    jacobian_tip_estimate_ = current_tip_pose_;
}

// Use the current joints and sample a desired tip pose close to the current tip pose
void CTRKinematics::publishCurrentJointAndDeltaTipPose(const ros::TimerEvent&)
{
    std::uniform_real_distribution<double> rotation(-50.0 * M_PI / 180, 50.0 * M_PI / 180);
    std::uniform_real_distribution<double> position(-0.01, 0.01);

    Robot_t::Vector6 rand_delta;
    rand_delta << position(re_), position(re_), position(re_), rotation(re_), rotation(re_), rotation(re_);
    Robot_t::Transform rand_transform;
    rand_transform = Robot_t::Transform::fromVector6_RPY(rand_delta);

    desired_tip_pose_ = current_tip_pose_ * rand_transform;
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
    desired_tip_pose_ = Robot_t::Transform::fromEigenMatrix(desired_tip_eigen.matrix());
}

// Main run function
void CTRKinematics::run()
{
    Real step_return;
    current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_, step_return);
    Robot_t::Vector6 current_tip_vector;
    current_tip_vector = current_tip_pose_.getVector6_RPY();

    Robot_t::Vector6 desired_tip_vector;
    desired_tip_vector = desired_tip_pose_.getVector6_RPY();

    Robot_t::Vector6 delta_tip_vector;
    Robot_t::Vector3 delta_translation;
    delta_translation = desired_tip_vector.head(3) - current_tip_vector.head(3);
    delta_tip_vector.setVector1(delta_translation);

    Robot_t::Vector3 delta_orientation;
    delta_orientation.setZero();
    // angles::shortest_angular_distance_with_limits(current_tip_vector.a(), desired_tip_vector.a(), -M_PI, M_PI, delta_orientation[0]);
    // angles::shortest_angular_distance_with_limits(current_tip_vector.b(), desired_tip_vector.b(), -M_PI, M_PI, delta_orientation[1]);
    angles::shortest_angular_distance_with_limits(current_tip_vector.c(), desired_tip_vector.c(), -M_PI, M_PI, delta_orientation[2]);

    Robot_t::Transform delta_tip_pose = current_tip_pose_.inv() * desired_tip_pose_;
    delta_tip_vector.setVector2(delta_orientation);

    // delta_tip_vector = delta_tip_pose.getVector6_RPY();
    ROS_INFO_STREAM("Position:\n" << delta_tip_vector.head(3));
    ROS_INFO_STREAM("Orientation:\n" << delta_tip_vector.tail(3));
    // Publish the cartesian and joint space error
    std_msgs::Float32 tip_error;
    tip_error.data = delta_tip_pose.getTranslation().norm();
    tip_error_pub_.publish(tip_error);
    std_msgs::Float32 joint_error;
    tip_error.data = (desired_joints_ - current_joints_).norm();
    joint_error_pub_.publish(tip_error);

    // compute the jacobian
    Robot_t::MatJacobian j_tip(6, 6);

    // Ensure current_joints_ (atleast one extension) is above a small value greater than 0 and below max extension
    if (current_joints_[1] <= 0) {
        current_joints_[1] = FLT_MIN;
    }
    if (current_joints_[1] >= 0.06647) {
        current_joints_[1] = 0.06647 - FLT_MIN;
    }
    if (current_joints_[4] <= 0) {
        current_joints_[4] = FLT_MIN;
    }
    if (current_joints_[4] >= 0.06647) {
        current_joints_[4] = 0.06647 - FLT_MIN;
    }
    c_robot_.calcJacobian(current_joints_, step_return, trans_diff_, rot_diff_, j_tip);
    //c_robot_.calcJacobian(current_joints_, current_tip_pose_, c_sample_, trans_diff, rot_diff, j_tip);

    // Change in joints
    Robot_t::VectorJ delta_q;

    // transpose method
    delta_q = j_tip.transpose() * delta_tip_vector;

    // psuedo inverse
    // delta_q = j_tip.completeOrthogonalDecomposition().pseudoInverse() * delta_tip_vector;

    // dls method
    //double lambda = 10; // set to zero for psuedo-inverse
    //Robot_t::MatJacobian j_damped(6,6);
    //j_damped = j_tip * j_tip.transpose() + pow(lambda, 2) * Robot_t::MatJacobian::Identity(6,6);
    //delta_q = j_tip.transpose() * j_damped.inverse() * (delta_tip_vector);

    // Limit velocity
    double ang_vel_limit = rot_diff_;
    double ext_vel_limit = trans_diff_;
    delta_q(0) = fmin(fmax(-ang_vel_limit, delta_q(0)), ang_vel_limit);
    delta_q(1) = fmin(fmax(-ext_vel_limit, delta_q(1)), ext_vel_limit);
    delta_q(2) = fmin(fmax(-ang_vel_limit, delta_q(2)), ang_vel_limit);
    delta_q(3) = fmin(fmax(-ang_vel_limit, delta_q(3)), ang_vel_limit);
    delta_q(4) = fmin(fmax(-ext_vel_limit, delta_q(4)), ext_vel_limit);
    delta_q(5) = fmin(fmax(-ang_vel_limit, delta_q(5)), ang_vel_limit);

    delta_q = delta_q.cwiseProduct(dof_index_);

    current_joints_+= delta_q;

    // Ensure current_joints_ (atleast one extension) is above a small value greater than 0 and below max extension
    if (current_joints_[1] <= 0) {
        current_joints_[1] = FLT_MIN;
    }
    if (current_joints_[1] >= 0.06647) {
        current_joints_[1] = 0.06647 - FLT_MIN;
    }
    if (current_joints_[4] <= 0) {
        current_joints_[4] = FLT_MIN;
    }
    if (current_joints_[4] >= 0.06647) {
        current_joints_[4] = 0.06647 - FLT_MIN;
    }
    // current_tip_pose_ = c_robot_.calcKinematic(current_joints_, c_sample_);

    // Compute tip estimate according to jacobian
    Robot_t::Vector6 delta_tip_estimate_vector;
    Robot_t::Transform delta_tip_estimate_transform;

    // Ensure jacobian_tip_estimate_current_joints_ (atleast one extension) is above a small value greater than 0 and below max extension
    jacobian_tip_estimate_current_joints_ += delta_q; // limit joint space for jacobian estimate.
    delta_tip_estimate_vector = j_tip * delta_q;
    delta_tip_estimate_transform = Robot_t::Transform::fromVector6_RPY(delta_tip_estimate_vector);
    jacobian_tip_estimate_.setTranslation(jacobian_tip_estimate_.getTranslation() + delta_tip_estimate_transform.getTranslation());
    jacobian_tip_estimate_.setRotation(delta_tip_estimate_transform.getRotation() * jacobian_tip_estimate_.getRotation());

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
    desired_tip_pose_viz_pub_.publish(TransformToPoseStamped(desired_tip_pose_));
    current_tip_pose_viz_pub_.publish(TransformToPoseStamped(current_tip_pose_));
    tip_pose_estimate_viz_pub_.publish(TransformToPoseStamped(jacobian_tip_estimate_));
}

geometry_msgs::PoseStamped CTRKinematics::TransformToPoseStamped(Robot_t::Transform &transform) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = transform.getX() * 100;
    pose_stamped.pose.position.y = transform.getY() * 100;
    pose_stamped.pose.position.z = transform.getZ() * 100;
    pose_stamped.pose.orientation.x = transform.getQuaternion().normalized().x();
    pose_stamped.pose.orientation.y = transform.getQuaternion().normalized().y();
    pose_stamped.pose.orientation.z = transform.getQuaternion().normalized().z();
    pose_stamped.pose.orientation.w = transform.getQuaternion().normalized().w();
    return pose_stamped;
}
