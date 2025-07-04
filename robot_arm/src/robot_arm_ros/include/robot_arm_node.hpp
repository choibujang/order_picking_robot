#ifndef ROBOT_ARM_NODE_HPP
#define ROBOT_ARM_NODE_HPP

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_arm_msgs/action/pick_and_place.hpp>

#include "i_arm_workflow_proxy.hpp"
#include "domain_types.hpp"
#include "robot_arm_core/include/arm_workflow_manager.hpp"
#include "robot_arm_core/include/controllers/arm_controller.h"
#include "robot_arm_core/include/controllers/cam_controller.hpp"
#include "robot_arm_core/include/controllers/net_controller.hpp"
#include "ros_interfaces/action/dispatch_manipulation_task.hpp"
#include "ros_interfaces/srv/get_detected_objects.hpp"

/**
 * @class RobotArmNode
 * @brief ROS 통신을 담당하고, 핵심 로직(ArmWorkflowManager)과 핵심 로직에서 사용되는 Controller들을 생성하고 소유한다.
 */
class RobotArmNode : public rclcpp::Node, public IArmWorkflowProxy {
public:
    explicit RobotArmNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RobotArmNode() override;

    // IArmWorkflowProxy interface
    std::future<ros_interfaces::srv::GetDetectedObjects::Response::SharedPtr> requestDetections() override;
    void publishFeedback(const std::string& status_message) override;
    void finalizeGoal(bool success, const std::string& final_message) override;
    bool isCancelRequested() override;
    void logInfo(const std::string& message) override;
    void logWarn(const std::string& message) override;
    void logError(const std::string& message) override;

private:
    using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
    using GoalHandleDispatchManipulationTask = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;

    void declare_parameters();
    void initialize_controllers();
    void initialize_ros_infra();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PickAndPDispatchManipulationTasklace::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle);

    // ROS
    rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
    rclcpp::Client<ros_interfaces::srv::GetDetectedObjects>::SharedPtr detect_objects_client_;
    rclcpp::TimerBase::SharedPtr fsm_timer_;
    std::shared_ptr<GoalHandleDispatchManipulationTask> goal_handle_;

    // 비즈니스 로직을 담당하는 클래스
    std::unique_ptr<ArmWorkflowManager> workflow_manager_;
    
    // ArmWorkflowManager에 pass될 controller들
    std::unique_ptr<ArmController> arm_controller_;
    std::unique_ptr<CamController> cam_controller_;
    std::unique_ptr<NetController> net_controller_;
};

#endif