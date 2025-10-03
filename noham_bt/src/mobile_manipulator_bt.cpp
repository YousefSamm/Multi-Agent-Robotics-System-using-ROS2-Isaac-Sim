#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "../include/noham_bt/behavior_tree_nodes.hpp"
#include "../include/noham_bt/mobile_manipulator_nodes.hpp"

class MobileManipulatorBehaviorTree : public rclcpp::Node
{
public:
    MobileManipulatorBehaviorTree() : Node("mobile_manipulator_behavior_tree")
    {
        // Initialize behavior tree
        initializeBehaviorTree();
        
        // Create timer to tick the behavior tree
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MobileManipulatorBehaviorTree::tickBehaviorTree, this));
            
        RCLCPP_INFO(this->get_logger(), "Mobile Manipulator Behavior Tree initialized");
    }

private:
    void initializeBehaviorTree()
    {
        // Create behavior tree factory
        factory_ = std::make_unique<BT::BehaviorTreeFactory>();
        
        // Register custom nodes with standard constructors
        factory_->registerNodeType<noham_bt::NavigateToPose>("NavigateToPose");
        factory_->registerNodeType<noham_bt::MobileManipulatorApproachNode>("MobileManipulatorApproach");
        factory_->registerNodeType<noham_bt::ArmPlanningNode>("ArmPlanning");
        factory_->registerNodeType<noham_bt::GripperActionNode>("GripperAction");
        
        // Load behavior tree from XML
        std::string package_path = ament_index_cpp::get_package_share_directory("noham_bt");
        std::string xml_path = std::filesystem::path(package_path) / "config" / "mobile_manipulator_tree.xml";
        
        if (!std::filesystem::exists(xml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Behavior tree XML file not found: %s", xml_path.c_str());
            return;
        }
        
        try {
            tree_ = factory_->createTreeFromFile(xml_path);
            RCLCPP_INFO(this->get_logger(), "Behavior tree loaded from: %s", xml_path.c_str());
            
            // Set parent node for all custom nodes
            setParentNodeForAllNodes();
            
            // Create console logger
            console_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            
            // Create Groot debugger publisher
            groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_);
            RCLCPP_INFO(this->get_logger(), "Groot debugger publisher created - connect to port 1667");
            
            RCLCPP_INFO(this->get_logger(), "Behavior tree ready to execute");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load behavior tree: %s", e.what());
            return;
        }
    }
    
    void setParentNodeForAllNodes()
    {
        RCLCPP_INFO(this->get_logger(), "Setting parent node for all behavior tree nodes...");
        
        // Get all nodes in the tree and set their parent node
        auto visitor = [this](BT::TreeNode* node) {
            if (auto navigate_node = dynamic_cast<noham_bt::NavigateToPose*>(node)) {
                RCLCPP_INFO(this->get_logger(), "Setting parent node for NavigateToPose node: %s", navigate_node->name().c_str());
                navigate_node->setParentNode(this);
            } else if (auto mobile_approach_node = dynamic_cast<noham_bt::MobileManipulatorApproachNode*>(node)) {
                RCLCPP_INFO(this->get_logger(), "Setting parent node for MobileManipulatorApproach node: %s", mobile_approach_node->name().c_str());
                mobile_approach_node->setParentNode(this);
            } else if (auto arm_planning_node = dynamic_cast<noham_bt::ArmPlanningNode*>(node)) {
                RCLCPP_INFO(this->get_logger(), "Setting parent node for ArmPlanning node: %s", arm_planning_node->name().c_str());
                arm_planning_node->setParentNode(this);
            } else if (auto gripper_action_node = dynamic_cast<noham_bt::GripperActionNode*>(node)) {
                RCLCPP_INFO(this->get_logger(), "Setting parent node for GripperAction node: %s", gripper_action_node->name().c_str());
                gripper_action_node->setParentNode(this);
            }
        };
        
        tree_.applyVisitor(visitor);
        RCLCPP_INFO(this->get_logger(), "Parent node setup completed for all behavior tree nodes");
    }
    
    void tickBehaviorTree()
    {
        if (!tree_.rootNode()) {
            return;
        }
        
        // Tick the behavior tree
        BT::NodeStatus status = tree_.tickOnce();
        
        // Add some status logging
        static int tick_count = 0;
        tick_count++;
        
        if (tick_count % 20 == 0) {  // Log every 10 seconds
            RCLCPP_INFO(this->get_logger(), "Behavior tree tick #%d, status: %s", 
                       tick_count, 
                       (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : 
                       (status == BT::NodeStatus::FAILURE) ? "FAILURE" : "RUNNING");
        }
        
        // Handle tree completion
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "✅ Mission completed! All mobile manipulator tasks finished successfully.");
            timer_->cancel();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "❌ Behavior tree failed!");
            timer_->cancel();
        }
    }
    
    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    BT::Tree tree_;
    std::unique_ptr<BT::StdCoutLogger> console_logger_;
    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MobileManipulatorBehaviorTree>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 Starting Mobile Manipulator Behavior Tree - Pick and Place Mission");
    RCLCPP_INFO(node->get_logger(), "📋 Mission: Navigate to AprilTag, pick object, and drop at target location");
    RCLCPP_INFO(node->get_logger(), "🔍 Groot debugger available on port 1667");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
