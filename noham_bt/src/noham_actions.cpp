#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <noham_custom_interfaces/action/forklift_approach.hpp>
#include <noham_custom_interfaces/action/lifter_action.hpp>

using namespace std::chrono_literals;
using ForkliftApproachAction = noham_custom_interfaces::action::ForkliftApproach;
using LifterActionAction = noham_custom_interfaces::action::LifterAction;

class NohamActionsNode : public rclcpp::Node
{
public:
  NohamActionsNode() : Node("noham_actions_node")
  {
    forklift_action_server_ = rclcpp_action::create_server<ForkliftApproachAction>(
      this, "forklift_approach",
      std::bind(&NohamActionsNode::handle_forklift_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NohamActionsNode::handle_forklift_cancel, this, std::placeholders::_1),
      std::bind(&NohamActionsNode::handle_forklift_accepted, this, std::placeholders::_1)
    );

    lifter_action_server_ = rclcpp_action::create_server<LifterActionAction>(
      this, "lifter_action",
      std::bind(&NohamActionsNode::handle_lifter_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NohamActionsNode::handle_lifter_cancel, this, std::placeholders::_1),
      std::bind(&NohamActionsNode::handle_lifter_accepted, this, std::placeholders::_1)
    );

    tag_subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "/tag_detections", 10, std::bind(&NohamActionsNode::tag_callback, this, std::placeholders::_1)
    );

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10, std::bind(&NohamActionsNode::camera_info_callback, this, std::placeholders::_1)
    );

    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&NohamActionsNode::joint_state_callback, this, std::placeholders::_1)
    );

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);
  }

private:
  rclcpp_action::GoalResponse handle_forklift_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ForkliftApproachAction::Goal>)
  {
    return camera_info_received_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_forklift_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ForkliftApproachAction>>)
  {
    auto stop = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_forklift_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<ForkliftApproachAction>> goal_handle)
  {
    std::thread{std::bind(&NohamActionsNode::execute_forklift, this, goal_handle)}.detach();
  }

  void execute_forklift(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ForkliftApproachAction>> goal_handle)
  {
    auto result = std::make_shared<ForkliftApproachAction::Result>();
    
    while (rclcpp::ok() && (target_centre_x_ == 0.0 && target_centre_y_ == 0.0)) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    
    while (rclcpp::ok() && (target_centre_x_ != 0.0 || target_centre_y_ != 0.0)) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = 0.5;
      twist.angular.z = -(target_centre_x_ - 320.0) * 0.01;
      cmd_vel_publisher_->publish(twist);
      
      std::this_thread::sleep_for(50ms);
    }
    
    auto stop = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop);
    
    auto forward_twist = geometry_msgs::msg::Twist();
    forward_twist.linear.x = 0.5;
    cmd_vel_publisher_->publish(forward_twist);
    
    std::this_thread::sleep_for(1000ms);
    
    stop = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop);
    
    result->success = true;
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse handle_lifter_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const LifterActionAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_lifter_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<LifterActionAction>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_lifter_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<LifterActionAction>> goal_handle)
  {
    std::thread{std::bind(&NohamActionsNode::execute_lifter, this, goal_handle)}.detach();
  }

  void execute_lifter(const std::shared_ptr<rclcpp_action::ServerGoalHandle<LifterActionAction>> goal_handle)
  {
    auto result = std::make_shared<LifterActionAction::Result>();
    double target_position = goal_handle->get_goal()->lift ? 0.2 : 0.0;
    
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->now();
    joint_msg.name = {"lift_joint"};
    joint_msg.position = {target_position};
    joint_publisher_->publish(joint_msg);
    
    while (rclcpp::ok()) {
      if (std::abs(current_lift_position_ - target_position) < 0.01) break;
      std::this_thread::sleep_for(100ms);
    }
    
    result->success = true;
    goal_handle->succeed(result);
  }

  void tag_callback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) {
      target_centre_x_ = 0.0;
      target_centre_y_ = 0.0;
    } else {
      const auto& detection = msg->detections[0];
      target_centre_x_ = detection.center.x;
      target_centre_y_ = detection.center.y;
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr)
  {
    if (!camera_info_received_) camera_info_received_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "lift_joint") {
        current_lift_position_ = msg->position[i];
        break;
      }
    }
  }

  rclcpp_action::Server<ForkliftApproachAction>::SharedPtr forklift_action_server_;
  rclcpp_action::Server<LifterActionAction>::SharedPtr lifter_action_server_;
  
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  
  bool camera_info_received_ = false;
  double current_lift_position_ = 0.0;
  double target_centre_x_ = 0.0;
  double target_centre_y_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NohamActionsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
