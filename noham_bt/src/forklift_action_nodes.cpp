#include "../include/noham_bt/forklift_action_nodes.hpp"
#include <chrono>

namespace noham_bt
{

// ForkliftApproach Node Implementation
ForkliftApproach::ForkliftApproach(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
    // Parent node will be set later via setParentNode
}

void ForkliftApproach::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
    
    if (parent_node_) {
        RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Parent node set successfully");
        
        // Create action client using the parent node
        action_client_ = rclcpp_action::create_client<noham_custom_interfaces::action::ForkliftApproach>(
            parent_node_, "forklift_approach");
        
        // Wait for action server
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(parent_node_->get_logger(), "ForkliftApproach: Action server not available");
        } else {
            RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Action server connected successfully");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ForkliftApproach"), "ForkliftApproach: Parent node is null");
    }
}

BT::PortsList ForkliftApproach::providedPorts()
{
    return {}; // No input ports needed
}

BT::NodeStatus ForkliftApproach::tick()
{
    if (!parent_node_) {
        RCLCPP_ERROR(rclcpp::get_logger("ForkliftApproach"), "ForkliftApproach: Parent node not set");
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if action server is available
    if (!action_client_ || !action_client_->action_server_is_ready()) {
        RCLCPP_WARN(parent_node_->get_logger(), "ForkliftApproach: Action server not ready, waiting...");
        return BT::NodeStatus::RUNNING;
    }
    
    if (!goal_sent_) {
        RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Starting forklift approach");
        
        // Create goal
        auto goal_msg = noham_custom_interfaces::action::ForkliftApproach::Goal();
        goal_msg.start = true;
        
        // Send goal
        auto send_goal_options = rclcpp_action::Client<noham_custom_interfaces::action::ForkliftApproach>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<noham_custom_interfaces::action::ForkliftApproach>::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
                goal_completed_ = true;
                RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Approach completed successfully");
            } else {
                RCLCPP_ERROR(parent_node_->get_logger(), "ForkliftApproach: Approach failed");
            }
        };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Approach goal sent");
    }
    
    // Check status
    if (goal_completed_) {
        RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Goal completed, returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(parent_node_->get_logger(), "ForkliftApproach: Goal in progress, returning RUNNING");
    return BT::NodeStatus::RUNNING;
}

void ForkliftApproach::halt()
{
    if (goal_sent_ && !goal_completed_ && action_client_) {
        action_client_->async_cancel_all_goals();
        RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Approach goal cancelled");
    }
    goal_sent_ = false;
    goal_completed_ = false;
    RCLCPP_INFO(parent_node_->get_logger(), "ForkliftApproach: Node halted");
}

// LifterAction Node Implementation
LifterAction::LifterAction(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
    // Parent node will be set later via setParentNode
}

void LifterAction::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
    
    if (parent_node_) {
        RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Parent node set successfully");
        
        // Create action client using the parent node
        action_client_ = rclcpp_action::create_client<noham_custom_interfaces::action::LifterAction>(
            parent_node_, "lifter_action");
        
        // Wait for action server
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(parent_node_->get_logger(), "LifterAction: Action server not available");
        } else {
            RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Action server connected successfully");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("LifterAction"), "LifterAction: Parent node is null");
    }
}

BT::PortsList LifterAction::providedPorts()
{
    return {
        BT::InputPort<bool>("lift")
    };
}

BT::NodeStatus LifterAction::tick()
{
    if (!parent_node_) {
        RCLCPP_ERROR(rclcpp::get_logger("LifterAction"), "LifterAction: Parent node not set");
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if action server is available
    if (!action_client_ || !action_client_->action_server_is_ready()) {
        RCLCPP_WARN(parent_node_->get_logger(), "LifterAction: Action server not ready, waiting...");
        return BT::NodeStatus::RUNNING;
    }
    
    if (!goal_sent_) {
        // Get lift parameter from input
        bool lift = true;
        if (!getInput("lift", lift)) {
            RCLCPP_ERROR(parent_node_->get_logger(), "LifterAction: Missing required input [lift]");
            return BT::NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Starting lifter action with lift=%s", lift ? "true" : "false");
        
        // Create goal
        auto goal_msg = noham_custom_interfaces::action::LifterAction::Goal();
        goal_msg.lift = lift;
        
        // Send goal
        auto send_goal_options = rclcpp_action::Client<noham_custom_interfaces::action::LifterAction>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<noham_custom_interfaces::action::LifterAction>::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
                goal_completed_ = true;
                RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Action completed successfully");
            } else {
                RCLCPP_ERROR(parent_node_->get_logger(), "LifterAction: Action failed");
            }
        };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Action goal sent with lift=%s", lift ? "true" : "false");
    }
    
    // Check status
    if (goal_completed_) {
        RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Goal completed, returning SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(parent_node_->get_logger(), "LifterAction: Goal in progress, returning RUNNING");
    return BT::NodeStatus::RUNNING;
}

void LifterAction::halt()
{
    if (goal_sent_ && !goal_completed_ && action_client_) {
        action_client_->async_cancel_all_goals();
        RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Action goal cancelled");
    }
    goal_sent_ = false;
    goal_completed_ = false;
    RCLCPP_INFO(parent_node_->get_logger(), "LifterAction: Node halted");
}

} // namespace noham_bt
