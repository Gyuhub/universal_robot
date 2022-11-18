#pragma once

// C++ STL
#include <iostream>
#include <vector>
#include <memory>

// Eigen
#include "eigen3/Eigen/Dense"

// ROS common
#include "ros/ros.h"

// ROS control
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"

// ROS messages
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

// ROS additional libraries
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_nr.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"
#include "pluginlib/class_list_macros.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_kdl/tf2_kdl.h"

namespace ur_example_controller
{

class CartesianPoseController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    CartesianPoseController();
    ~CartesianPoseController();

    bool init(hardware_interface::PositionJointInterface *robot_hw, ros::NodeHandle &nh) override;
    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;

private:
    // publishers and subscribers
    realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> ee_state_pub_;
    realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> ee_state_dot_pub_;
    ros::Subscriber command_sub_;

    // kdl library configuration
    KDL::Tree ur5_tree_;
    KDL::Chain ur5_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_rec_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_pinv_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_pos_nr;
    std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_;

    std::shared_ptr<KDL::JntArray> jnt_pos_start_;
    std::shared_ptr<KDL::JntArrayVel> jnt_vel_start_;
    std::shared_ptr<KDL::Frame> ee_pos_goal_;
    std::shared_ptr<KDL::Jacobian> jac_;
    std::shared_ptr<Eigen::VectorXd> ee_vel_;

    // ros control configuration
    std::vector<hardware_interface::JointHandle> joint_handle_;
    int joint_num_;
    std::string base_frame_id_, ee_frame_id_;
    geometry_msgs::PoseStamped ee_state_, ee_desired_;
    geometry_msgs::TwistStamped ee_state_dot_;

    // callback function for command subscriber
    void GetDesiredEndEffectorPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

} // namespace ur_example_controller
