#ifndef ROBOT_ARM_NODE_HPP
#define ROBOT_ARM_NODE_HPP

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "ros_interfaces/action/dispatch_manipulation_task.hpp"
#include "ros_interfaces/srv/get_detected_objects.hpp"
#include "robot_arm_controllers/cam/cam_controller.hpp"
#include "robot_arm_controllers/arm/arm_controller.h"
#include "robot_arm_controllers/net/net_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/**
 * @class RobotArmNode
 * @brief Service Server로부터 Request를 받아 실행하는 dispatch_manipulation_task 액션 서버와
 *        /detected_object 토픽을 listen하는 subscriber로 이루어져 있다.
 */
class RobotArmNode : public rclcpp::Node
{
public:
  using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
  using GoalHandleDMT = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;
  using GetDetectedObjects = ros_interfaces::srv::GetDetectedObjects;

  explicit RobotArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_arm_node", options)
  {
      using namespace std::placeholders;

      this->action_server_ = rclcpp_action::create_server<DispatchManipulationTask>(
      this,
      "dispatch_manipulation_task",
      std::bind(&RobotArmNode::handle_goal, this, _1, _2),
      std::bind(&RobotArmNode::handle_cancel, this, _1),
      std::bind(&RobotArmNode::handle_accepted, this, _1));

      this->service_client_ = this->create_client<GetDetectedObjects>("get_detected_objects");
  }

private:
  /**
   * @brief goal의 item_name 배열 길이와 item_quantities 배열 길이가 다르면 Reject한다.
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DispatchManipulationTask::Goal> goal);
  
  
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDMT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleDMT> goal_handle);


  /**
   * @brief 주어진 물체 목록을 지정된 수량만큼 집어서 정해진 위치에 놓는 액션을 수행한다.
   * 
   * @details
   * string[] item_names와 int32[] item_quantities를 입력으로 받아,  
   * item_names[i]에 해당하는 물체를 item_quantities[i]개만큼 pick하고
   * 정해진 위치에 place한다.
   */
  void execute(const std::shared_ptr<GoalHandleDMT> goal_handle);


  // std::vector<std::vector<float>> convertTo3DCoords(const std::vector<DetectedObject>& objects, const cv::Mat& depth_map);


  void startSendFrameThread() {
    running_ = true;
    consecutive_net_failures_ = false;
    consecutive_cam_failures_ = false;

    send_frame_thread_ = std::thread([this]() {
      cam_controller_.startCameraPipeline();

      while (running_) {
        if(!cam_controller_.getFrameSet()) {
          consecutive_cam_failures_ = true;
        }

        auto color_frame = cam_controller_.getColorFrame();
        
        if (!net_controller_.sendMjpegData(color_frame)) {
          consecutive_net_failures_ = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  void stopSendFrameThread() {
    running_ = false;
    consecutive_net_failures_ = false;
    consecutive_cam_failures_ = false;
    cam_controller_.stopCameraPipeline();

    if (send_frame_thread_.joinable()) send_frame_thread_.join();
  }

  void printDetectedObjects(const std::shared_ptr<ros_interfaces::srv::GetDetectedObjects::Response>& response);


  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  rclcpp::Client<GetDetectedObjects>::SharedPtr service_client_;

  ArmController arm_controller_;
  CamController cam_controller_;
  NetController net_controller_;
  
  std::thread send_frame_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<bool> running_ = false;
  std::atomic<bool> consecutive_net_failures_ = false;
  std::atomic<bool> consecutive_cam_failures_ = false;
  std::atomic<bool> abort_requested_ = false;
  
};

#endif