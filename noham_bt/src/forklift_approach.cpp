#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <noham_custom_interfaces/action/forklift_approach.hpp>
#include <memory>
#include <chrono>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;
using ForkliftApproachAction = noham_custom_interfaces::action::ForkliftApproach;
using GoalHandleForkliftApproach = rclcpp_action::ServerGoalHandle<ForkliftApproachAction>;

class ForkliftApproachNode : public rclcpp::Node
{
public:
  ForkliftApproachNode() : Node("forklift_approach_node")
  {
    action_server_ = rclcpp_action::create_server<ForkliftApproachAction>(
      this,
      "forklift_approach",
      std::bind(&ForkliftApproachNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ForkliftApproachNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ForkliftApproachNode::handle_accepted, this, std::placeholders::_1)
    );

    tag_subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "/tag_detections", 10, std::bind(&ForkliftApproachNode::tag_callback, this, std::placeholders::_1)
    );

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10, std::bind(&ForkliftApproachNode::camera_info_callback, this, std::placeholders::_1)
    );

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    is_active_ = false;
    image_width_ = 640;
    image_height_ = 480;
    target_centre_x_ = 0.0;
    target_centre_y_ = 0.0;
    target_area_ = 0.0;
    camera_info_received_ = false;
    current_goal_handle_ = nullptr;
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ForkliftApproachAction::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    
    if (!camera_info_received_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    if (is_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleForkliftApproach> goal_handle)
  {
    (void)goal_handle;
    
    auto stop_twist = geometry_msgs::msg::Twist();
    stop_twist.linear.x = 0.0;
    stop_twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(stop_twist);
    
    is_active_ = false;
    current_goal_handle_ = nullptr;
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(std::shared_ptr<GoalHandleForkliftApproach> goal_handle)
  {
    std::thread{std::bind(&ForkliftApproachNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleForkliftApproach> goal_handle)
  {
    current_goal_handle_ = goal_handle;
    is_active_ = true;
    
    auto result = std::make_shared<ForkliftApproachAction::Result>();
    auto feedback = std::make_shared<ForkliftApproachAction::Feedback>();
    
    // Wait for initial AprilTag detection
    while (rclcpp::ok() && is_active_ && (target_centre_x_ == 0.0 && target_centre_y_ == 0.0)) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      
      if (!goal_handle->is_active()) {
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(100ms);
    }
    
    // Main execution loop - runs until AprilTag is no longer detected
    while (rclcpp::ok() && is_active_) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      
      if (!goal_handle->is_active()) {
        result->success = false;
        goal_handle->abort(result);
        return;
      }
      
      goal_handle->publish_feedback(feedback);
      
      // Check if AprilTag is no longer detected (coordinates reset to 0)
      if (target_centre_x_ == 0.0 && target_centre_y_ == 0.0) {
        auto stop_twist = geometry_msgs::msg::Twist();
        stop_twist.linear.x = 0.0;
        stop_twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_twist);
        
        result->success = true;
        goal_handle->succeed(result);
        break;
      }
      
      move_towards_tag();
      std::this_thread::sleep_for(50ms);
    }
    
    current_goal_handle_ = nullptr;
    is_active_ = false;
  }

  void tag_callback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (!is_active_ || !camera_info_received_) {
      return;
    }

    if (msg->detections.empty()) {
      // Only reset coordinates if we were previously tracking a tag
      if (target_centre_x_ != 0.0 || target_centre_y_ != 0.0) {
        target_centre_x_ = 0.0;
        target_centre_y_ = 0.0;
      }
      return;
    }

    const auto& detection = msg->detections[0];
    target_centre_x_ = detection.center.x;
    target_centre_y_ = detection.center.y;
    
    if (detection.corners.size() >= 4) {
      double min_x = detection.corners[0].x;
      double max_x = detection.corners[0].x;
      double min_y = detection.corners[0].y;
      double max_y = detection.corners[0].y;
      
      for (const auto& corner : detection.corners) {
        min_x = std::min(min_x, corner.x);
        max_x = std::max(max_x, corner.x);
        min_y = std::min(min_y, corner.y);
        max_y = std::max(max_y, corner.y);
      }
      
      target_area_ = (max_x - min_x) * (max_y - min_y);
    } else {
      target_area_ = 100.0;
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!camera_info_received_) {
      image_width_ = msg->width;
      image_height_ = msg->height;
      camera_info_received_ = true;
    }
  }

  void move_towards_tag()
  {
    auto twist = geometry_msgs::msg::Twist();
    
    double linear_speed = 0.5;
    double angular_gain = 0.01;
    double max_angular_speed = 1.0;
    
    double error_x = target_centre_x_ - (image_width_ / 2.0);
    
    twist.linear.x = linear_speed;
    twist.angular.z = -error_x * angular_gain;
    
    twist.angular.z = std::clamp(twist.angular.z, -max_angular_speed, max_angular_speed);
    
    if (std::abs(twist.angular.z) < 0.001) {
      twist.angular.z = 0.0;
    }
    
    cmd_vel_publisher_->publish(twist);
  }

  rclcpp_action::Server<ForkliftApproachAction>::SharedPtr action_server_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  bool is_active_;
  int image_width_;
  int image_height_;
  double target_centre_x_;
  double target_centre_y_;
  double target_area_;
  bool camera_info_received_;
  std::shared_ptr<GoalHandleForkliftApproach> current_goal_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForkliftApproachNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
