#include "../include/noham_bt/behavior_tree_nodes.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace noham_bt
{

// NavigateToPose Node Implementation
NavigateToPose::NavigateToPose(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
    // Parent node will be set later via setParentNode
}

void NavigateToPose::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
    
    if (parent_node_) {
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Parent node set successfully");
        
        // Create action client using the parent node
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            parent_node_, "navigate_to_pose");
        
        // Wait for action server
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(parent_node_->get_logger(), "NavigateToPose: Action server not available");
        } else {
            RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Action server connected successfully");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "NavigateToPose: Parent node is null");
    }
}

BT::PortsList NavigateToPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("target_pose")
    };
}

BT::NodeStatus NavigateToPose::tick()
{
    if (!parent_node_) {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "NavigateToPose: Parent node not set - this should not happen");
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if action server is available
    if (!action_client_ || !action_client_->action_server_is_ready()) {
        RCLCPP_WARN(parent_node_->get_logger(), "NavigateToPose: Action server not ready, waiting...");
        return BT::NodeStatus::RUNNING;
    }
    
    if (!goal_sent_) {
        // Get target pose from input
        if (!getInput("target_pose", target_pose_param_)) {
            RCLCPP_ERROR(parent_node_->get_logger(), "NavigateToPose: Missing required input [target_pose]");
            return BT::NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Starting navigation to pose: %s", target_pose_param_.c_str());
        
        // Parse target pose (format: "x,y,z,roll,pitch,yaw")
        std::stringstream ss(target_pose_param_);
        std::string item;
        std::vector<double> pose_values;
        
        while (std::getline(ss, item, ',')) {
            pose_values.push_back(std::stod(item));
        }
        
        if (pose_values.size() != 6) {
            RCLCPP_ERROR(parent_node_->get_logger(), "NavigateToPose: Invalid pose format. Expected: x,y,z,roll,pitch,yaw");
            return BT::NodeStatus::FAILURE;
        }
        
        // Create goal
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        
        goal_msg.pose.pose.position.x = pose_values[0];
        goal_msg.pose.pose.position.y = pose_values[1];
        goal_msg.pose.pose.position.z = pose_values[2];
        
        // Convert Euler angles to quaternion
        tf2::Quaternion q;
        q.setRPY(pose_values[3], pose_values[4], pose_values[5]);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();
        
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = parent_node_->now();
        
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Sending goal to position (%.4f, %.4f, %.4f) with yaw %.4f", 
                   pose_values[0], pose_values[1], pose_values[2], pose_values[5]);
        
        // Send goal
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                goal_completed_ = true;
                RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Navigation completed successfully");
            } else {
                RCLCPP_ERROR(parent_node_->get_logger(), "NavigateToPose: Navigation failed with code: %d", static_cast<int>(result.code));
            }
        };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Navigation goal sent to: %s", target_pose_param_.c_str());
    }
    
    // Check status
    if (goal_completed_) {
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Goal completed, returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(parent_node_->get_logger(), "NavigateToPose: Goal in progress, returning RUNNING");
    return BT::NodeStatus::RUNNING;
}

void NavigateToPose::halt()
{
    if (goal_sent_ && !goal_completed_ && action_client_) {
        action_client_->async_cancel_all_goals();
        RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Navigation goal cancelled");
    }
    goal_sent_ = false;
    goal_completed_ = false;
    RCLCPP_INFO(parent_node_->get_logger(), "NavigateToPose: Node halted");
}

// RotateInPlace Node Implementation
RotateInPlace::RotateInPlace(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), rotation_started_(false)
{
    // Parent node will be set later via setParentNode
}

void RotateInPlace::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
    
    if (parent_node_) {
        RCLCPP_INFO(parent_node_->get_logger(), "RotateInPlace: Parent node set successfully");
        
        // Create publisher for cmd_vel using the parent node
        cmd_vel_pub_ = parent_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Set rotation speed (rad/s)
        rotation_speed_ = 0.5;
        
        RCLCPP_INFO(parent_node_->get_logger(), "RotateInPlace: cmd_vel publisher created");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("RotateInPlace"), "RotateInPlace: Parent node is null");
    }
}

BT::PortsList RotateInPlace::providedPorts()
{
    return {
        BT::InputPort<double>("rotation_angle")
    };
}

BT::NodeStatus RotateInPlace::tick()
{
    if (!parent_node_) {
        RCLCPP_ERROR(rclcpp::get_logger("RotateInPlace"), "Parent node not set");
        return BT::NodeStatus::FAILURE;
    }
    
    if (!rotation_started_) {
        // Get rotation angle from input (default to 2π = full rotation)
        double rotation_angle = -6.28; // -2*PI = 360 degrees
        getInput("rotation_angle", rotation_angle);
        
        // Calculate rotation time
        rotation_duration_ = std::chrono::duration<double>(rotation_angle / rotation_speed_);
        start_time_ = parent_node_->now();
        rotation_started_ = true;
        
        RCLCPP_INFO(parent_node_->get_logger(), "RotateInPlace: Starting full rotation (%.2f seconds)", rotation_duration_.count());
    }
    
    // Check if rotation is complete
    auto elapsed_time = parent_node_->now() - start_time_;
    if (elapsed_time.seconds() >= rotation_duration_.count()) {
        // Stop rotation
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        
        RCLCPP_INFO(parent_node_->get_logger(), "RotateInPlace: Full rotation completed, returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    
    // Continue rotation
    geometry_msgs::msg::Twist rotate_cmd;
    rotate_cmd.angular.z = rotation_speed_;
    cmd_vel_pub_->publish(rotate_cmd);
    
    RCLCPP_DEBUG(parent_node_->get_logger(), "RotateInPlace: Rotation in progress (%.1f/%.1f seconds)", 
                 elapsed_time.seconds(), rotation_duration_.count());
    return BT::NodeStatus::RUNNING;
}

void RotateInPlace::halt()
{
    // Stop rotation
    if (cmd_vel_pub_) {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
    }
    
    rotation_started_ = false;
    RCLCPP_INFO(parent_node_->get_logger(), "RotateInPlace: Rotation halted");
}

} // namespace noham_bt
