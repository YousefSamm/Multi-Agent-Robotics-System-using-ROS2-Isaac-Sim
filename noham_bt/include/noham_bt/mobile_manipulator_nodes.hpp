#ifndef NOHAM_BT_MOBILE_MANIPULATOR_NODES_HPP_
#define NOHAM_BT_MOBILE_MANIPULATOR_NODES_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <noham_custom_interfaces/action/mobile_manipulator_approach.hpp>
#include <noham_custom_interfaces/action/arm_planning.hpp>
#include <noham_custom_interfaces/action/gripper_action.hpp>

namespace noham_bt
{

class MobileManipulatorApproachNode : public BT::ActionNodeBase
{
public:
    MobileManipulatorApproachNode(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<noham_custom_interfaces::action::MobileManipulatorApproach>::SharedPtr action_client_;
    bool goal_sent_;
    bool goal_completed_;
};

class ArmPlanningNode : public BT::ActionNodeBase
{
public:
    ArmPlanningNode(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<noham_custom_interfaces::action::ArmPlanning>::SharedPtr action_client_;
    bool goal_sent_;
    bool goal_completed_;
};

class GripperActionNode : public BT::ActionNodeBase
{
public:
    GripperActionNode(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<noham_custom_interfaces::action::GripperAction>::SharedPtr action_client_;
    bool goal_sent_;
    bool goal_completed_;
};

} // namespace noham_bt

#endif // NOHAM_BT_MOBILE_MANIPULATOR_NODES_HPP_
