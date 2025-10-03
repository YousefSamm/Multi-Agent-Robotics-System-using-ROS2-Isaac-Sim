#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <noham_custom_interfaces/action/lifter_action.hpp>
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using LifterActionAction = noham_custom_interfaces::action::LifterAction;
using GoalHandleLifterAction = rclcpp_action::ServerGoalHandle<LifterActionAction>;

class LifterActionNode : public rclcpp::Node
{
public:
    LifterActionNode() : Node("lifter_action_node")
    {
        // Action server for lift/drop commands
        action_server_ = rclcpp_action::create_server<LifterActionAction>(
            this,
            "lifter_action",
            std::bind(&LifterActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LifterActionNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&LifterActionNode::handle_accepted, this, std::placeholders::_1)
        );

        // Publisher for joint commands
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        // Publisher for operation completion status
        completion_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/lifter_completion", 10);

        // Subscriber for joint state feedback
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&LifterActionNode::joint_state_callback, this, std::placeholders::_1));

        // Initialize state
        current_lift_position_ = 0.0;
        target_position_ = 0.0;
        is_moving_ = false;
        last_command_ = false;
        current_goal_handle_ = nullptr;
        is_active_ = false;

        RCLCPP_INFO(this->get_logger(), "Lifter action node started");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const LifterActionAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        
        if (is_active_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLifterAction> goal_handle)
    {
        (void)goal_handle;
        
        is_active_ = false;
        is_moving_ = false;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<GoalHandleLifterAction> goal_handle)
    {
        std::thread{std::bind(&LifterActionNode::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLifterAction> goal_handle)
    {
        current_goal_handle_ = goal_handle;
        is_active_ = true;
        
        auto result = std::make_shared<LifterActionAction::Result>();
        auto feedback = std::make_shared<LifterActionAction::Feedback>();
        
        bool lift_command = goal_handle->get_goal()->lift;
        
        if (lift_command != last_command_) {
            if (lift_command) {
                target_position_ = 1.0; // Lift up
            } else {
                target_position_ = 0.0; // Drop down
            }
            
            last_command_ = lift_command;
            is_moving_ = true;
            
            // Send joint command
            auto joint_msg = sensor_msgs::msg::JointState();
            joint_msg.header.stamp = this->now();
            joint_msg.name = {"lift_joint"};
            joint_msg.position = {target_position_};
            joint_msg.velocity = {0.0};
            joint_msg.effort = {0.0};
            
            joint_pub_->publish(joint_msg);
        }
        
        // Wait for movement to complete
        while (rclcpp::ok() && is_active_ && is_moving_) {
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
            
            // Check if movement is complete
            if (std::abs(current_lift_position_ - target_position_) < 0.01) {
                is_moving_ = false;
                break;
            }
            
            std::this_thread::sleep_for(100ms);
        }
        
        if (is_active_) {
            result->success = true;
            goal_handle->succeed(result);
        }
        
        current_goal_handle_ = nullptr;
        is_active_ = false;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find the lifter joint
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "lift_joint") {
                current_lift_position_ = msg->position[i];
                break;
            }
        }
        
        // Publish completion status
        auto completion_msg = std_msgs::msg::Bool();
        completion_msg.data = !is_moving_;
        completion_pub_->publish(completion_msg);
    }

    rclcpp_action::Server<LifterActionAction>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    // State variables
    double current_lift_position_;
    double target_position_;
    bool is_moving_;
    bool last_command_;
    std::shared_ptr<GoalHandleLifterAction> current_goal_handle_;
    bool is_active_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifterActionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
