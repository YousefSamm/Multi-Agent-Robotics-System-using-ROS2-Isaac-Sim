#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <noham_custom_interfaces/action/arm_planning.hpp>
#include <noham_custom_interfaces/action/gripper_action.hpp>
#include <noham_custom_interfaces/action/mobile_manipulator_approach.hpp>

using namespace std::chrono_literals;
using ArmPlanningAction = noham_custom_interfaces::action::ArmPlanning;
using GripperActionAction = noham_custom_interfaces::action::GripperAction;
using MobileManipulatorApproachAction = noham_custom_interfaces::action::MobileManipulatorApproach;

class ManipulatorControlNode : public rclcpp::Node
{
public:
  ManipulatorControlNode() : Node("manipulator_control_node")
  {
    // Create action servers
    arm_planning_server_ = rclcpp_action::create_server<ArmPlanningAction>(
      this, "arm_planning",
      std::bind(&ManipulatorControlNode::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManipulatorControlNode::handle_arm_cancel, this, std::placeholders::_1),
      std::bind(&ManipulatorControlNode::handle_arm_accepted, this, std::placeholders::_1)
    );

    gripper_action_server_ = rclcpp_action::create_server<GripperActionAction>(
      this, "gripper_action",
      std::bind(&ManipulatorControlNode::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManipulatorControlNode::handle_gripper_cancel, this, std::placeholders::_1),
      std::bind(&ManipulatorControlNode::handle_gripper_accepted, this, std::placeholders::_1)
    );

    mobile_manipulator_approach_server_ = rclcpp_action::create_server<MobileManipulatorApproachAction>(
      this, "mobile_manipulator_approach",
      std::bind(&ManipulatorControlNode::handle_mobile_approach_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManipulatorControlNode::handle_mobile_approach_cancel, this, std::placeholders::_1),
      std::bind(&ManipulatorControlNode::handle_mobile_approach_accepted, this, std::placeholders::_1)
    );

    // Subscribe to joint states for gripper feedback
    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/isaac_joint_states", 10, std::bind(&ManipulatorControlNode::joint_state_callback, this, std::placeholders::_1)
    );

      // Subscribe to AprilTag detections and camera info for mobile approach
  tag_subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
    "/tag_detections", 10, std::bind(&ManipulatorControlNode::tag_callback, this, std::placeholders::_1)
  );

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10, std::bind(&ManipulatorControlNode::camera_info_callback, this, std::placeholders::_1)
    );

    // Publisher for gripper commands
    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_command", 10);

    // Publisher for mobile base movement
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Wait for TF to be available
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(this->get_logger(), "Manipulator control node initialized");
  }

private:
  // Arm Planning Action Handlers
  rclcpp_action::GoalResponse handle_arm_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmPlanningAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_arm_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmPlanningAction>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_arm_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmPlanningAction>> goal_handle)
  {
    std::thread{std::bind(&ManipulatorControlNode::execute_arm_planning, this, goal_handle)}.detach();
  }

  void execute_arm_planning(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ArmPlanningAction>> goal_handle)
  {
    auto result = std::make_shared<ArmPlanningAction::Result>();
    std::string command = goal_handle->get_goal()->command;
    
    RCLCPP_INFO(this->get_logger(), "Executing arm command: %s", command.c_str());

    try {
      using moveit::planning_interface::MoveGroupInterface;
      auto move_group_interface = MoveGroupInterface(this->shared_from_this(), "arm");
      
      if (command == "pick") {
        // Get transform from base to dp and execute pick
        geometry_msgs::msg::Pose target_pose;
        
        try {
          auto transform = tf_buffer_->lookupTransform(
            "j2n6s300_link_base", 
            "tag36h11:1", 
            tf2::TimePointZero
          );
          
          // Set position from transform with Z offset
          target_pose.position.x = transform.transform.translation.x- 0.018;
          target_pose.position.y = transform.transform.translation.y;
          target_pose.position.z = transform.transform.translation.z + 0.04; // Add 0.05 to Z
          
          // Keep the same orientation
          target_pose.orientation.x = 0.7744;
          target_pose.orientation.y = -0.0421;
          target_pose.orientation.z = 0.6286;
          target_pose.orientation.w = 0.0384;
          
          RCLCPP_INFO(this->get_logger(), "Pick pose set to dp position with Z offset: x=%.3f, y=%.3f, z=%.3f", 
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
        } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
          RCLCPP_INFO(this->get_logger(), "Using default pick pose instead");
          
          // Fallback to default pose if TF fails
          target_pose.position.x = 0.2;
          target_pose.position.y = 0.0;
          target_pose.position.z = 0.5;
          target_pose.orientation.x = 0.7744;
          target_pose.orientation.y = -0.0421;
          target_pose.orientation.z = 0.6286;
          target_pose.orientation.w = 0.0384;
        }

        move_group_interface.setPoseTarget(target_pose);
        
      } else if (command == "drop") {
        // Set drop position
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.1;
        target_pose.position.y = 0.5;
        target_pose.position.z = 0.0;
        target_pose.orientation.x = 0.7744;
        target_pose.orientation.y = -0.0421;
        target_pose.orientation.z = 0.6286;
        target_pose.orientation.w = 0.0384;
        
        move_group_interface.setPoseTarget(target_pose);
        
      } else if (command == "init_pos") {
        // Go to initial positions
        std::vector<double> joint_positions = {0.0, 2.0, 1.57, 0.0, 2.0, 0.0};
        move_group_interface.setJointValueTarget(joint_positions);
        
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown arm command: %s", command.c_str());
        result->success = false;
        goal_handle->succeed(result);
        return;
      }

      // Create and execute plan
      auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      if (success) {
        move_group_interface.execute(plan);
        result->success = true;
        RCLCPP_INFO(this->get_logger(), "Arm command '%s' executed successfully", command.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed for command: %s", command.c_str());
        result->success = false;
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during arm execution: %s", e.what());
      result->success = false;
    }

    goal_handle->succeed(result);
  }

  // Gripper Action Handlers
  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperActionAction::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperActionAction>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperActionAction>> goal_handle)
  {
    std::thread{std::bind(&ManipulatorControlNode::execute_gripper_action, this, goal_handle)}.detach();
  }

  void execute_gripper_action(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperActionAction>> goal_handle)
  {
    auto result = std::make_shared<GripperActionAction::Result>();
    std::string command = goal_handle->get_goal()->command;
    
    RCLCPP_INFO(this->get_logger(), "Executing gripper command: %s", command.c_str());

    if (command == "grasp") {
      // Set gripper to grasp position (2.0 radians)
      auto joint_msg = sensor_msgs::msg::JointState();
      joint_msg.header.stamp = this->now();
      joint_msg.name = {
        "j2n6s300_joint_finger_tip_1",
        "j2n6s300_joint_finger_tip_2", 
        "j2n6s300_joint_finger_tip_3"
      };
      joint_msg.position = {2.0, 2.0, 2.0};
      
      RCLCPP_INFO(this->get_logger(), "Publishing gripper command: grasp at position 2.0");
      joint_publisher_->publish(joint_msg);
      
      // Wait for gripper to reach position with timeout
      auto start_time = this->now();
      auto timeout = rclcpp::Duration::from_seconds(10.0); // 10 second timeout
      
      while (rclcpp::ok()) {
        // Check timeout
        if (this->now() - start_time > timeout) {
          RCLCPP_WARN(this->get_logger(), "Gripper grasp timeout reached");
          result->success = false;
          goal_handle->succeed(result);
          return;
        }
        
        // Check if joints reached target position
        bool reached = true;
        for (const auto& joint_name : joint_msg.name) {
          auto it = std::find(current_joint_names_.begin(), current_joint_names_.end(), joint_name);
          if (it != current_joint_names_.end()) {
            size_t index = std::distance(current_joint_names_.begin(), it);
            if (index < current_joint_positions_.size()) {
              double current_pos = current_joint_positions_[index];
              if (std::abs(current_pos - 2.0) > 0.1) {
                reached = false;
                RCLCPP_DEBUG(this->get_logger(), "Joint %s at %.3f, target 2.0", joint_name.c_str(), current_pos);
                break;
              }
            } else {
              reached = false;
              break;
            }
          } else {
            reached = false;
            RCLCPP_DEBUG(this->get_logger(), "Joint %s not found in current joint states", joint_name.c_str());
            break;
          }
        }
        
        if (reached) {
          RCLCPP_INFO(this->get_logger(), "All gripper joints reached grasp position");
          break;
        }
        
        std::this_thread::sleep_for(100ms);
      }
      
      result->success = true;
      RCLCPP_INFO(this->get_logger(), "Grasp command executed successfully");
      
    } else if (command == "drop") {
      // Set gripper to open position (0.0 radians)
      auto joint_msg = sensor_msgs::msg::JointState();
      joint_msg.header.stamp = this->now();
      joint_msg.name = {
        "j2n6s300_joint_finger_tip_1",
        "j2n6s300_joint_finger_tip_2", 
        "j2n6s300_joint_finger_tip_3"
      };
      joint_msg.position = {0.0, 0.0, 0.0};
      
      RCLCPP_INFO(this->get_logger(), "Publishing gripper command: drop at position 0.0");
      joint_publisher_->publish(joint_msg);
      
      // Wait for gripper to reach position with timeout
      auto start_time = this->now();
      auto timeout = rclcpp::Duration::from_seconds(10.0); // 10 second timeout
      
      while (rclcpp::ok()) {
        // Check timeout
        if (this->now() - start_time > timeout) {
          RCLCPP_WARN(this->get_logger(), "Gripper drop timeout reached");
          result->success = false;
          goal_handle->succeed(result);
          return;
        }
        
        // Check if joints reached target position
        bool reached = true;
        for (const auto& joint_name : joint_msg.name) {
          auto it = std::find(current_joint_names_.begin(), current_joint_names_.end(), joint_name);
          if (it != current_joint_names_.end()) {
            size_t index = std::distance(current_joint_names_.begin(), it);
            if (index < current_joint_positions_.size()) {
              double current_pos = current_joint_positions_[index];
              if (std::abs(current_pos - 0.0) > 0.01) {
                reached = false;
                RCLCPP_DEBUG(this->get_logger(), "Joint %s at %.3f, target 0.0", joint_name.c_str(), current_pos);
                break;
              }
            } else {
              reached = false;
              break;
            }
          } else {
            reached = false;
            RCLCPP_DEBUG(this->get_logger(), "Joint %s not found in current joint states", joint_name.c_str());
            break;
          }
        }
        
        if (reached) {
          RCLCPP_INFO(this->get_logger(), "All gripper joints reached drop position");
          break;
        }
        
        std::this_thread::sleep_for(100ms);
      }
      
      result->success = true;
      RCLCPP_INFO(this->get_logger(), "Drop command executed successfully");
      
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown gripper command: %s", command.c_str());
      result->success = false;
    }

    goal_handle->succeed(result);
  }

  // Mobile Manipulator Approach Action Handlers
  rclcpp_action::GoalResponse handle_mobile_approach_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MobileManipulatorApproachAction::Goal>)
  {
    return camera_info_received_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::CancelResponse handle_mobile_approach_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MobileManipulatorApproachAction>>)
  {
    auto stop = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_mobile_approach_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<MobileManipulatorApproachAction>> goal_handle)
  {
    std::thread{std::bind(&ManipulatorControlNode::execute_mobile_approach, this, goal_handle)}.detach();
  }

  void execute_mobile_approach(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MobileManipulatorApproachAction>> goal_handle)
  {
    auto result = std::make_shared<MobileManipulatorApproachAction::Result>();
    
    RCLCPP_INFO(this->get_logger(), "Starting mobile manipulator approach to AprilTag");
    
    // Wait for AprilTag to be detected
    while (rclcpp::ok() && (target_centre_x_ == 0.0 && target_centre_y_ == 0.0)) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
    
    RCLCPP_INFO(this->get_logger(), "AprilTag detected at center: (%.1f, %.1f)", target_centre_x_, target_centre_y_);
    
    // Approach the AprilTag while keeping it centered
    while (rclcpp::ok() && (target_centre_x_ != 0.0 || target_centre_y_ != 0.0)) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }
      
      // Move towards the tag while keeping it centered
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = 0.3; // Slower approach speed for precision
      twist.angular.z = -(target_centre_x_ - 320.0) * 0.01; // Keep tag centered (assuming 640x480 camera)
      cmd_vel_publisher_->publish(twist);
      
      std::this_thread::sleep_for(50ms);
    }
    
    // Check if we lost the tag (which means we're very close or passed it)
    if (target_centre_x_ == 0.0 && target_centre_y_ == 0.0) {
      RCLCPP_INFO(this->get_logger(), "AprilTag ID 4 no longer detected - continuing forward for 3.0 seconds");
      
      // Continue moving forward for half a second at the same speed
      auto forward_twist = geometry_msgs::msg::Twist();
      forward_twist.linear.x = 0.3; // Same speed as approach
      forward_twist.angular.z = 0.0; // No rotation, just straight forward
      cmd_vel_publisher_->publish(forward_twist);
      
      // Wait for half a second
      std::this_thread::sleep_for(4000ms);
    }
    
    // Stop the robot
    auto stop = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop);
    
    result->success = true;
    RCLCPP_INFO(this->get_logger(), "Mobile manipulator approach completed successfully");
    goal_handle->succeed(result);
  }



  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_names_ = msg->name;
    current_joint_positions_ = msg->position;
    
    // Debug: Log when we receive joint states
    static int counter = 0;
    if (++counter % 100 == 0) { // Log every 100th message to avoid spam
      RCLCPP_DEBUG(this->get_logger(), "Received joint states with %zu joints", msg->name.size());
      for (size_t i = 0; i < msg->name.size() && i < 5; ++i) { // Log first 5 joints
        RCLCPP_DEBUG(this->get_logger(), "  %s: %.3f", msg->name[i].c_str(), msg->position[i]);
      }
    }
  }

  void tag_callback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    target_centre_x_ = 0.0;
    target_centre_y_ = 0.0;
    
    // Look for AprilTag with ID = 4
    for (const auto& detection : msg->detections) {
      if (detection.id == 4) {
        target_centre_x_ = detection.center.x;
        target_centre_y_ = detection.center.y;
        RCLCPP_DEBUG(this->get_logger(), "AprilTag ID 4 detected at center: (%.1f, %.1f)", target_centre_x_, target_centre_y_);
        break;
      }
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr)
  {
    if (!camera_info_received_) camera_info_received_ = true;
  }

  // Action servers
  rclcpp_action::Server<ArmPlanningAction>::SharedPtr arm_planning_server_;
  rclcpp_action::Server<GripperActionAction>::SharedPtr gripper_action_server_;
  rclcpp_action::Server<MobileManipulatorApproachAction>::SharedPtr mobile_manipulator_approach_server_;
  
  // Subscriptions and publishers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
      // Joint state tracking
    std::vector<std::string> current_joint_names_;
    std::vector<double> current_joint_positions_;
    
    // Mobile approach tracking
    bool camera_info_received_ = false;
    double target_centre_x_ = 0.0;
    double target_centre_y_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulatorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
