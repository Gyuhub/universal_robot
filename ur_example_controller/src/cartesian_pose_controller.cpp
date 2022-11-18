#include "ur_example_controller/cartesian_pose_controller.h"
#include "pluginlib/class_list_macros.hpp"

namespace ur_example_controller
{
CartesianPoseController::CartesianPoseController()
{
}

CartesianPoseController::~CartesianPoseController()
{
}

bool CartesianPoseController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &nh)
{
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names) || joint_names.size() != 6)
    {
        ROS_ERROR("Not appropriate joint names are found! Please check the name of the joints again");
        return false;
    }
    joint_num_ = joint_names.size();
    std::string robot_description;
    if (!nh.getParam("/robot_description", robot_description))
    {
        ROS_ERROR("Failed to get the robot_description!");
        return false;
    }
    for (unsigned int i = 0; i < joint_num_; i++)
    {
        try
        {
            joint_handle_.push_back(robot->getHandle(joint_names[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }
    }
    if (!nh.getParam("base_frame_id", base_frame_id_))
    {
        ROS_ERROR("Failed to get the name of the base!");
        return false;        
    }
    if (!nh.getParam("ee_frame_id", ee_frame_id_))
    {
        ROS_ERROR("Failed to get the name of the end-effecotr!");
        return false;        
    }

    // get kinematic tree from robot_description
    if (!kdl_parser::treeFromString(robot_description, ur5_tree_))
    {
        ROS_ERROR("Failed to parse the tree from robot_description!");
        return false;
    }

    // get kinematic chain from tree
    if (!ur5_tree_.getChain(base_frame_id_, ee_frame_id_, ur5_chain_))
    {
        ROS_ERROR("Failed to get the chain from tree!");
        return false;
    }

    // create solvers
    fk_solver_pos_rec_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(ur5_chain_);
    ik_solver_vel_pinv_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(ur5_chain_, 0.0001, 1000);
    ik_solver_pos_nr = std::make_unique<KDL::ChainIkSolverPos_NR>(ur5_chain_, *fk_solver_pos_rec_, *ik_solver_vel_pinv_, 1000);
    jnt_to_jac_ = std::make_shared<KDL::ChainJntToJacSolver>(ur5_chain_);

    // initialize configuration
    jnt_pos_start_ = std::make_shared<KDL::JntArray>(joint_num_);
    jnt_vel_start_ = std::make_shared<KDL::JntArrayVel>(joint_num_);
    ee_vel_ = std::make_shared<Eigen::VectorXd>(6);
    jac_ = std::make_shared<KDL::Jacobian>(joint_num_);

    // initialize publishers and subscribers
    ee_state_pub_.init(nh, "ee_state", 1);
    ee_state_dot_pub_.init(nh, "ee_state_dot", 1);
    ee_state_.header.frame_id = base_frame_id_;
    ee_state_.header.stamp = ros::Time::now();
    ee_state_dot_.header.frame_id = base_frame_id_;
    ee_state_dot_.header.stamp = ros::Time::now();
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> lock(ee_state_pub_);
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> lock_dot(ee_state_dot_pub_);
    command_sub_ = nh.subscribe("/ee_pose_desired", 1, &CartesianPoseController::GetDesiredEndEffectorPose, this);

    ROS_INFO("Successfully initialize the controller!");
    return true;
}

void CartesianPoseController::starting(const ros::Time &time)
{
    // get initial position of each joint
    for (int i = 0; i < joint_num_; i++)
    {
        jnt_pos_start_->data(i) = joint_handle_[i].getPosition();
        jnt_vel_start_->qdot.data(i) = joint_handle_[i].getVelocity();
    }
    ROS_INFO("Initial joint configuration of the UR5: %f %f %f %f %f %f", jnt_pos_start_->data(0),
                                                                          jnt_pos_start_->data(1),
                                                                          jnt_pos_start_->data(2),
                                                                          jnt_pos_start_->data(3),
                                                                          jnt_pos_start_->data(4),
                                                                          jnt_pos_start_->data(5));
    // get initial cartesian position and convert current joint position to end-effector's postion
    KDL::Frame ee_pos_start;
    fk_solver_pos_rec_->JntToCart(*jnt_pos_start_, ee_pos_start);
    ee_pos_goal_ = std::make_shared<KDL::Frame>(ee_pos_start);
    jnt_to_jac_->JntToJac(*jnt_pos_start_, *jac_);
    *ee_vel_ = jac_->data * jnt_vel_start_->qdot.data;
    return;
}

void CartesianPoseController::update(const ros::Time &time, const ros::Duration &period)
{
    // get position of each joint
    for (int i = 0; i < joint_num_; i++)
    {
        jnt_pos_start_->data(i) = joint_handle_[i].getPosition();
        jnt_vel_start_->qdot.data(i) = joint_handle_[i].getVelocity();
    }

    // convert current joint position to end-effector's postion
    KDL::Frame ee_pos_start;
    fk_solver_pos_rec_->JntToCart(*jnt_pos_start_, ee_pos_start);

    // compute inverse kinematics
    KDL::JntArray jnt_pos_goal(joint_num_);
    ik_solver_pos_nr->CartToJnt(*jnt_pos_start_, *ee_pos_goal_, jnt_pos_goal);

    // compute jacobian
    jnt_to_jac_->JntToJac(*jnt_pos_start_, *jac_);
    *ee_vel_ = jac_->data * jnt_vel_start_->qdot.data;

    // publish states of the robot
    geometry_msgs::TransformStamped tf_stamped = tf2::kdlToTransform(ee_pos_start);
    ee_state_.pose.position.x = tf_stamped.transform.translation.x;
    ee_state_.pose.position.y = tf_stamped.transform.translation.y;
    ee_state_.pose.position.z = tf_stamped.transform.translation.z;
    ee_state_.pose.orientation = tf_stamped.transform.rotation;
    if (ee_state_pub_.trylock())
    {
        ee_state_pub_.msg_ = ee_state_;
        ee_state_pub_.unlockAndPublish();
    }

    // publish derivate of states of the robot
    ee_state_dot_.header.stamp = ros::Time::now();
    ee_state_dot_.twist.linear.x = ee_vel_->head(3)(0);
    ee_state_dot_.twist.linear.y = ee_vel_->head(3)(1);
    ee_state_dot_.twist.linear.z = ee_vel_->head(3)(2);
    ee_state_dot_.twist.angular.x = ee_vel_->tail(3)(0);
    ee_state_dot_.twist.angular.y = ee_vel_->tail(3)(1);
    ee_state_dot_.twist.angular.z = ee_vel_->tail(3)(2);
    if (ee_state_dot_pub_.trylock())
    {
        ee_state_dot_pub_.msg_ = ee_state_dot_;
        ee_state_dot_pub_.unlockAndPublish();
    }

    // set joint command
    for (int i = 0; i < joint_num_; i++)
    {
        joint_handle_[i].setCommand(jnt_pos_goal.data(i));
    }
    return;
}

void CartesianPoseController::GetDesiredEndEffectorPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ee_desired_ = *msg;
    geometry_msgs::TransformStamped t;
    t.header.frame_id = base_frame_id_;
    t.child_frame_id = ee_frame_id_;
    t.transform.translation.x = ee_desired_.pose.position.x;
    t.transform.translation.y = ee_desired_.pose.position.y;
    t.transform.translation.z = ee_desired_.pose.position.z;
    t.transform.rotation = ee_desired_.pose.orientation;
    *ee_pos_goal_ = tf2::transformToKDL(t);
    return;
}


} // namespace ur_example_controller

PLUGINLIB_EXPORT_CLASS(ur_example_controller::CartesianPoseController, controller_interface::ControllerBase)