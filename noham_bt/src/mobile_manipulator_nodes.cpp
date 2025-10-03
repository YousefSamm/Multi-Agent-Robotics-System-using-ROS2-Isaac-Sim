#include "../include/noham_bt/mobile_manipulator_nodes.hpp"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

namespace noham_bt
{

// MobileManipulatorApproachNode implementation
MobileManipulatorApproachNode::MobileManipulatorApproachNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
}

BT::PortsList MobileManipulatorApproachNode::providedPorts()
{
    return {};
}

BT::NodeStatus MobileManipulatorApproachNode::tick()
{
    if (!parent_node_) {
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<noham_custom_interfaces::action::MobileManipulatorApproach>(
            parent_node_, "mobile_manipulator_approach");
    }

    if (!goal_sent_) {
        auto goal_msg = noham_custom_interfaces::action::MobileManipulatorApproach::Goal();
        goal_msg.start = true;

        auto send_goal_options = rclcpp_action::Client<noham_custom_interfaces::action::MobileManipulatorApproach>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<noham_custom_interfaces::action::MobileManipulatorApproach>::WrappedResult&) {
            goal_completed_ = true;
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "Mobile manipulator approach goal sent");
        return BT::NodeStatus::RUNNING;
    }

    if (goal_completed_) {
        goal_sent_ = false;
        goal_completed_ = false;
        RCLCPP_INFO(parent_node_->get_logger(), "Mobile manipulator approach completed");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void MobileManipulatorApproachNode::halt()
{
    goal_sent_ = false;
    goal_completed_ = false;
}

void MobileManipulatorApproachNode::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
}

// ArmPlanningNode implementation
ArmPlanningNode::ArmPlanningNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
}

BT::PortsList ArmPlanningNode::providedPorts()
{
    return {BT::InputPort<std::string>("command")};
}

BT::NodeStatus ArmPlanningNode::tick()
{
    if (!parent_node_) {
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<noham_custom_interfaces::action::ArmPlanning>(
            parent_node_, "arm_planning");
    }

    if (!goal_sent_) {
        std::string command;
        if (!getInput("command", command)) {
            RCLCPP_ERROR(parent_node_->get_logger(), "ArmPlanning: Missing required input [command]");
            return BT::NodeStatus::FAILURE;
        }
        
        auto goal_msg = noham_custom_interfaces::action::ArmPlanning::Goal();
        goal_msg.command = command;

        auto send_goal_options = rclcpp_action::Client<noham_custom_interfaces::action::ArmPlanning>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<noham_custom_interfaces::action::ArmPlanning>::WrappedResult&) {
            goal_completed_ = true;
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "Arm planning goal sent: %s", command.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (goal_completed_) {
        goal_sent_ = false;
        goal_completed_ = false;
        RCLCPP_INFO(parent_node_->get_logger(), "Arm planning completed successfully");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void ArmPlanningNode::halt()
{
    goal_sent_ = false;
    goal_completed_ = false;
}

void ArmPlanningNode::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
}

// GripperActionNode implementation
GripperActionNode::GripperActionNode(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config), parent_node_(nullptr), goal_sent_(false), goal_completed_(false)
{
}

BT::PortsList GripperActionNode::providedPorts()
{
    return {BT::InputPort<std::string>("command")};
}

BT::NodeStatus GripperActionNode::tick()
{
    if (!parent_node_) {
        return BT::NodeStatus::FAILURE;
    }

    if (!action_client_) {
        action_client_ = rclcpp_action::create_client<noham_custom_interfaces::action::GripperAction>(
            parent_node_, "gripper_action");
    }

    if (!goal_sent_) {
        std::string command;
        if (!getInput("command", command)) {
            RCLCPP_ERROR(parent_node_->get_logger(), "GripperAction: Missing required input [command]");
            return BT::NodeStatus::FAILURE;
        }
        
        auto goal_msg = noham_custom_interfaces::action::GripperAction::Goal();
        goal_msg.command = command;

        auto send_goal_options = rclcpp_action::Client<noham_custom_interfaces::action::GripperAction>::SendGoalOptions();
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<noham_custom_interfaces::action::GripperAction>::WrappedResult&) {
            goal_completed_ = true;
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
        RCLCPP_INFO(parent_node_->get_logger(), "Gripper action goal sent: %s", command.c_str());
        return BT::NodeStatus::RUNNING;
    }

    if (goal_completed_) {
        goal_sent_ = false;
        goal_completed_ = false;
        RCLCPP_INFO(parent_node_->get_logger(), "Gripper action completed successfully");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void GripperActionNode::halt()
{
    goal_sent_ = false;
    goal_completed_ = false;
}

void GripperActionNode::setParentNode(rclcpp::Node* parent_node)
{
    parent_node_ = parent_node;
}

} // namespace noham_bt
