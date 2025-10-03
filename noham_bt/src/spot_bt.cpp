#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "../include/noham_bt/behavior_tree_nodes.hpp"

class SpotBehaviorTree : public rclcpp::Node
{
public:
    SpotBehaviorTree() : Node("spot_behavior_tree")
    {
        // Declare parameters for room poses
        this->declare_parameter("room1_pose", "14.1706,-6.96892,0.0,0.0,0.0,0.0");
        this->declare_parameter("room2_pose", "14.2639,7.78589,0.0,0.0,0.0,-1.53049");
        this->declare_parameter("room3_pose", "25.3004,0.391258,0.0,0.0,0.0,-3.13383");
        
        // Get parameters
        room1_pose_ = this->get_parameter("room1_pose").as_string();
        room2_pose_ = this->get_parameter("room2_pose").as_string();
        room3_pose_ = this->get_parameter("room3_pose").as_string();
        
        // Initialize behavior tree
        initializeBehaviorTree();
        
        // Create timer to tick the behavior tree
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpotBehaviorTree::tickBehaviorTree, this));
            
        RCLCPP_INFO(this->get_logger(), "Spot Behavior Tree initialized");
        RCLCPP_INFO(this->get_logger(), "Room 1: %s", room1_pose_.c_str());
        RCLCPP_INFO(this->get_logger(), "Room 2: %s", room2_pose_.c_str());
        RCLCPP_INFO(this->get_logger(), "Room 3: %s", room3_pose_.c_str());
    }

private:
    void initializeBehaviorTree()
    {
        // Create behavior tree factory
        factory_ = std::make_unique<BT::BehaviorTreeFactory>();
        
        // Register custom nodes with standard constructors
        factory_->registerNodeType<noham_bt::NavigateToPose>("NavigateToPose");
        factory_->registerNodeType<noham_bt::RotateInPlace>("RotateInPlace");
        
        // Load behavior tree from XML
        std::string package_path = ament_index_cpp::get_package_share_directory("noham_bt");
        std::string xml_path = std::filesystem::path(package_path) / "config" / "room_exploration_tree.xml";
        
        if (!std::filesystem::exists(xml_path)) {
            RCLCPP_ERROR(this->get_logger(), "Behavior tree XML file not found: %s", xml_path.c_str());
            return;
        }
        
        try {
            tree_ = factory_->createTreeFromFile(xml_path);
            RCLCPP_INFO(this->get_logger(), "Behavior tree loaded from: %s", xml_path.c_str());
            
            // Set blackboard values for room poses
            tree_.rootBlackboard()->set("room1_pose", room1_pose_);
            tree_.rootBlackboard()->set("room2_pose", room2_pose_);
            tree_.rootBlackboard()->set("room3_pose", room3_pose_);
            
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
            } else if (auto rotate_node = dynamic_cast<noham_bt::RotateInPlace*>(node)) {
                RCLCPP_INFO(this->get_logger(), "Setting parent node for RotateInPlace node: %s", rotate_node->name().c_str());
                rotate_node->setParentNode(this);
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
            RCLCPP_INFO(this->get_logger(), "✅ Mission completed! All rooms visited and rotations done.");
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
    
    std::string room1_pose_;
    std::string room2_pose_;
    std::string room3_pose_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SpotBehaviorTree>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 Starting Spot Behavior Tree - Room Navigation Mission");
    RCLCPP_INFO(node->get_logger(), "📋 Mission: Navigate to 3 rooms with full rotation at each");
    RCLCPP_INFO(node->get_logger(), "🔍 Groot debugger available on port 1667");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
