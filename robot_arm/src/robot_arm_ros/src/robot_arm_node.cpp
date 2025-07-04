#include "robot_arm_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

RobotArmNode::RobotArmNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("robot_arm_node", options) {
    declare_parameters();
    initialize_controllers();
    initialize_ros_infra();

    workflow_manager_ = std::make_unique<ArmWorkflowManager>(
        *this,
        std::move(arm_controller_),
        std::move(cam_controller_),
        std::move(net_controller_));
    
    RCLCPP_INFO(this->get_logger(), "RobotArmNode has been initialized.");
}

RobotArmNode::~RobotArmNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down RobotArmNode.");
    if (fsm_timer_) {
        fsm_timer_->cancel();
    }
}

void RobotArmNode::declare_parameters() {
    this->declare_parameter<int>("device_id", 0);
    this->declare_parameter<std::string>("server_ip", "127.0.0.1");
    this->declare_parameter<int>("server_port", 8080);
}

void RobotArmNode::initialize_controllers() {
    RCLCPP_INFO(this->get_logger(), "Initializing controllers...");
    try {
        arm_controller_ = std::make_unique<ArmController>();
        cam_controller_ = std::make_unique<CamController>(this->get_parameter("device_id").as_int());
        net_controller_ = std::make_unique<NetController>();
    } catch (const std::runtime_error& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize controllers: %s", e.what());
        throw;
    }
    RCLCPP_INFO(this->get_logger(), "Controllers initialized successfully.");
}

void RobotArmNode::initialize_ros_infra() {
    RCLCPP_INFO(this->get_logger(), "Initializing ROS infrastructure...");
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<DispatchManipulationTask>(
        this,
        "dispatch_manipulation_task",
        std::bind(&RobotArmNode::handle_goal, this, _1, _2),
        std::bind(&RobotArmNode::handle_cancel, this, _1),
        std::bind(&RobotArmNode::handle_accepted, this, _1));

    detect_objects_client_ = this->create_client<ros_interfaces::srv::GetDetectedObjects>("detect_objects");

    fsm_timer_ = this->create_wall_timer(100ms, [this]() {
        if (workflow_manager_) {
            workflow_manager_->tick();
        }
    });

    RCLCPP_INFO(this->get_logger(), "ROS infrastructure initialized successfully.");
}

rclcpp_action::GoalResponse RobotArmNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const PickAndPlace::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with %ld items", goal->items.size());
    // TODO: goal의 형식이 적절한지 체크하는 로직 추가할것
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotArmNode::handle_cancel(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotArmNode::handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Starting workflow.");
    std::vector<arm_core::GoalItem> core_goals;
    for (const auto& ros_goal_item : goal_handle->get_goal()->items) {
        core_goals.push_back({ros_goal_item.name, ros_goal_item.quantity});
    }

    this->goal_handle_ = goal_handle;
    workflow_manager_->start(core_goals);
}

// IArmWorkflowProxy interface implementation
std::future<ros_interfaces::srv::GetDetectedObjects::Response::SharedPtr> RobotArmNode::requestDetections() {
    auto request = std::make_shared<ros_interfaces::srv::GetDetectedObjects::Request>();

    return detect_objects_client_->async_send_request(request);
}

void RobotArmNode::publishFeedback(const std::string& status_message) {
    auto feedback = std::make_shared<PickAndPlace::Feedback>();
    feedback->status = status_message;
    goal_handle_->publish_feedback(feedback);
}

void RobotArmNode::finalizeGoal(bool success, const std::string& final_message) {
    auto result = std::make_shared<PickAndPlace::Result>();
    result->success = success;
    result->message = final_message;
    if (success) {
        goal_handle_->succeed(result);
    } else {
        goal_handle_->abort(result);
    }

    goal_handle_.reset();
}

bool RobotArmNode::isCancelRequested() {
    if (goal_handle_) {
        return goal_handle_->is_cancel_requested();
    }
    return false;
}

void RobotArmNode::logInfo(const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "[Workflow] %s", message.c_str());
}

void RobotArmNode::logWarn(const std::string& message) {
    RCLCPP_WARN(this->get_logger(), "[Workflow] %s", message.c_str());
}

void RobotArmNode::logError(const std::string& message) {
    RCLCPP_ERROR(this->get_logger(), "[Workflow] %s", message.c_str());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 