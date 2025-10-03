#ifndef NOHAM_BT_FORKLIFT_ACTION_NODES_HPP_
#define NOHAM_BT_FORKLIFT_ACTION_NODES_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <noham_custom_interfaces/action/forklift_approach.hpp>
#include <noham_custom_interfaces/action/lifter_action.hpp>

namespace noham_bt
{

class ForkliftApproach : public BT::ActionNodeBase
{
public:
    ForkliftApproach(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<noham_custom_interfaces::action::ForkliftApproach>::SharedPtr action_client_;
    bool goal_sent_;
    bool goal_completed_;
};

class LifterAction : public BT::ActionNodeBase
{
public:
    LifterAction(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<noham_custom_interfaces::action::LifterAction>::SharedPtr action_client_;
    bool goal_sent_;
    bool goal_completed_;
};

} // namespace noham_bt

#endif // NOHAM_BT_FORKLIFT_ACTION_NODES_HPP_
