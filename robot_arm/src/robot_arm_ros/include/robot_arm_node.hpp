#ifndef ROBOT_ARM_NODE_HPP
#define ROBOT_ARM_NODE_HPP

#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

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
 * @brief dispatch_manipulation_task 액션 서버와
 *        get_detected_objects 서비스 클라이언트로 이루어져 있다.
 */
class RobotArmNode : public rclcpp::Node
{
public:
  using DispatchManipulationTask = ros_interfaces::action::DispatchManipulationTask;
  using GoalHandleDMT = rclcpp_action::ServerGoalHandle<DispatchManipulationTask>;
  using GetDetectedObjects = ros_interfaces::srv::GetDetectedObjects;

  explicit RobotArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_arm_node", options),
    device_id_(1), server_ip_("127.0.0.1"), server_port_(8080),
    arm_controller_(ArmController(true)),
    net_controller_(NetController(device_id_, server_ip_, server_port_))
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
   * item_names[i]에 해당하는 물체를 item_quantities[i]개만큼 pick하고
   * 정해진 위치에 place한다.
   */
  void execute(const std::shared_ptr<GoalHandleDMT> goal_handle);


  /**
   * @brief
   *  카메라에서 RGB 이미지를 주기적으로 받아와 AI 서버로 전송하는 별도의 스레드를 시작한다.
   */
  void startSendFrameThread() {
    running_ = true;

    send_frame_thread_ = std::thread([this]() {
      cam_controller_.start();

      while (running_) {
        if(!cam_controller_.update()) {
          continue;
        }

        auto color_frame = cam_controller_.getColorFrame();
        
        net_controller_.sendMjpegData(color_frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  void stopSendFrameThread() {
    running_ = false;
    cam_controller_.stop();

    if (send_frame_thread_.joinable()) send_frame_thread_.join();
  }

  /**
   * @brief
   *  ros_interfaces::srv::GetDetectedObjects::Response 타입의
   *  string[] class_names, int32[] counts, int32[] pixel_x, int32[] pixel_y, int32[] pixel_class_indices
   *  형태의 서비스 응답을 받아,
   *  각 클래스별로 이름, 개수, 픽셀 좌표 목록을
   *  예시와 같이 보기 좋게 출력하는 함수.
   *
   *  예시 출력:
   *    Class: apple (count=2)
   *      - (x1, y1)
   *      - (x2, y2)
   *    Class: pear (count=1)
   *      - (x3, y3)
   *
   * @param response 서비스 응답 객체
   */
  void printDetectedObjects(const std::shared_ptr<ros_interfaces::srv::GetDetectedObjects::Response>& response);


  rclcpp_action::Server<DispatchManipulationTask>::SharedPtr action_server_;
  rclcpp::Client<GetDetectedObjects>::SharedPtr service_client_;

  const int device_id_;
  const std::string server_ip_;
  const int server_port_;

  ArmController arm_controller_;
  CamController cam_controller_;
  NetController net_controller_;
  
  std::thread send_frame_thread_;
  std::atomic<bool> running_ = false;


  
};

#endif