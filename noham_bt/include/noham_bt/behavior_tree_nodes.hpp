#ifndef NOHAM_BT_BEHAVIOR_TREE_NODES_HPP_
#define NOHAM_BT_BEHAVIOR_TREE_NODES_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace noham_bt
{

class NavigateToPose : public BT::ActionNodeBase
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    std::string target_pose_param_;
    bool goal_sent_;
    bool goal_completed_;
};

class RotateInPlace : public BT::ActionNodeBase
{
public:
    RotateInPlace(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    
    // Set the parent node after construction
    void setParentNode(rclcpp::Node* parent_node);

private:
    rclcpp::Node* parent_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double rotation_speed_;
    std::chrono::duration<double> rotation_duration_;
    rclcpp::Time start_time_;
    bool rotation_started_;
};

// Forward declarations for the new action nodes
class ForkliftApproach;
class LifterAction;

} // namespace noham_bt

#endif // NOHAM_BT_BEHAVIOR_TREE_NODES_HPP_
