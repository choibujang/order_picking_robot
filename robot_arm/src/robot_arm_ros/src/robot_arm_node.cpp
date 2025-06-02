#include "robot_arm_node.hpp"

rclcpp_action::GoalResponse RobotArmNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const DispatchManipulationTask::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with items:");
  for (const std::string& item : goal->item_names) {
    RCLCPP_INFO(this->get_logger(), " %s", item.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "with quantities:");
  for (const int& quantity : goal->item_quantities) {
    RCLCPP_INFO(this->get_logger(), " %d", quantity);
  }

  (void)uuid;

  if (goal->item_names.size() != goal->item_quantities.size()) {
    RCLCPP_WARN(this->get_logger(), "Rejected goal: mismatched name/quantity sizes");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotArmNode::handle_cancel(
  const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotArmNode::handle_accepted(const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&RobotArmNode::execute, this, _1), goal_handle}.detach();
}

void RobotArmNode::execute(const std::shared_ptr<GoalHandleDMT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
  auto result = std::make_shared<DispatchManipulationTask::Result>();

  // AI 서버로 영상 전송 시작
  startSendFrameThread();
  RCLCPP_INFO(this->get_logger(), "Start Send Frame Thread");

  // AI 서버에 서비스 요청
  int retries = 3;

  while (retries-- > 0 && !service_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for service to appear... (%d retries left)", retries);
  }
  
  if (retries < 0) {
    stopSendFrameThread();
    result->success = false;
    result->error_code = 3;
    result->error_msg = "Cannot find /get_detected_objects service."
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Goal aborted");
    return;
  }

  auto request = std::make_shared<GetDetectedObjects::Request>();
  auto future = service_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5)) 
      == rclcpp::FutureReturnCode::SUCCESS) {

    auto response = future.get();
    printDetectedObjects(response);
  } else if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_WARN(this->get_logger(), "Timed out waiting for /get_detected_objects service response.");
  } else {
      RCLCPP_ERROR(this->get_logger(), "Service call interrupted or failed.");
  }


  status_timer_.reset();
  stopSendFrameThread();
 
  
}
 

void RobotArmNode::printDetectedObjects(const std::shared_ptr<ros_interfaces::srv::GetDetectedObjects::Response>& response)
{
    const auto& class_names = response->class_names;
    const auto& counts = response->counts;
    const auto& pixel_x = response->pixel_x;
    const auto& pixel_y = response->pixel_y;
    const auto& pixel_class_indices = response->pixel_class_indices;

    struct ObjectInfo {
        std::string name;
        int count;
        std::vector<std::pair<int, int>> pixels;
    };

    std::unordered_map<int, ObjectInfo> objects;

    // class_names[i]는 index i에 해당함
    for (size_t i = 0; i < class_names.size(); ++i) {
        objects[i] = {
            class_names[i],
            counts[i],
            {}
        };
    }

    // 각 픽셀 좌표들을 class_idx 기준으로 묶기
    for (size_t i = 0; i < pixel_class_indices.size(); ++i) {
        int class_idx = pixel_class_indices[i];
        int x = pixel_x[i];
        int y = pixel_y[i];
        objects[class_idx].pixels.emplace_back(x, y);
    }

    // 출력
    for (const auto& [idx, info] : objects) {
        std::cout << "Class: " << info.name << " (count=" << info.count << ")\n";
        for (const auto& [x, y] : info.pixels) {
            std::cout << "  - (" << x << ", " << y << ")\n";
        }
    }
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotArmNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}