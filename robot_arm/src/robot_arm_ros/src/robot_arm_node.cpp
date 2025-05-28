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

  // AI 서버로 이미지 전송 시작, while로 SendFrameThread내의 이상 감지.
  startSendFrameThread();

  while (rclcpp::ok()) {
    if (consecutive_cam_failures_) {
      RCLCPP_ERROR(this->get_logger(), "Camera Failed 5 times");
      stopSendFrameThread();
      result->success = false;
      result->error_code = 1;
      result->error_msg = "Camera Failure.";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      return;
    }

    if (consecutive_net_failures_) {
      RCLCPP_ERROR(this->get_logger(), "Network Failed 5 times");
      stopSendFrameThread();
      result->success = false;
      result->error_code = 2;
      result->error_msg = "Network Failure";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // AI 서버에 서비스 요청. 3회 이상 응답이 없을 경우 에러 처리
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
    result->error_msg = "/get_detected_objects service not available.";
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Goal aborted");
    return;
  }

  auto request = std::make_shared<GetDetectedObjects::Request>();
  auto future = service_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(2)) 
      == rclcpp::FutureReturnCode::SUCCESS) {

    auto response = future.get();
    printDetectedObjects(response);
  }


  status_timer_.reset();
  stopSendFrameThread();
 
  






  //   bool goal_met = false;

  //   std::lock_guard<std::mutex> lock(detected_mutex_);

  //   // goal 항목들이 latest_detected_objects_에 다 있는지 확인
  //   goal_met = true;
  //   for (size_t i = 0; i < goal->item_names.size(); ++i) {
  //     const std::string& name = goal->item_names[i];
  //     int required_count = goal->item_quantities[i];
  //     int detected_count = 0;

  //     for (const auto& obj : latest_detected_objects_) {
  //       if (obj.name == name) {
  //         detected_count += obj.count;
  //       }
  //     }

  //     if (detected_count < required_count) {
  //       goal_met = false;
  //       break;
  //     }
  //   }
  
  //   if (goal_met) {
  //     confirmed_objects = latest_detected_objects_;
  //     RCLCPP_INFO(this->get_logger(), "All target items detected.");
  //     break;
  //   }
  
  //   rclcpp::Rate(5).sleep();
  // }

  // cv::Mat depth_data = cam_controller_.getDepthData()
  // cv::Mat depth_map = cam_controller_.createDepthMap(depth_data);

  // vector<vector<float>> comfirmed_objects_3d = convertTo3DCoords(confirmed_objects, depth_map);




  



  // for (int i = 1; (i < size(goal->item_names)) && rclcpp::ok(); i++) {
  //   // Check if there is a cancel request
  //   if (goal_handle->is_canceling()) {
  //     result->success = false;
  //     result->error_code = 100;
  //     result->error_msg = "client cancel";
  //     goal_handle->canceled(result);
  //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //     return;
  //   }



  //   feedback->progress = i;
  //   // Publish feedback
  //   goal_handle->publish_feedback(feedback);
  //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

  //   loop_rate.sleep();
  // }

  // // Check if goal is done
  // if (rclcpp::ok()) {
  //   result->success = true;
  //   result->error_code = 0;
  //   result->error_msg = "";
  //   goal_handle->succeed(result);
  //   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  // }
}
 


// std::vector<std::vector<float>> RobotArmNode::convertTo3DCoords(const std::vector<DetectedObject>& objects, const cv::Mat& depth_map) {
//   std::vector<std::vector<float>> result;

//   for (const auto& obj : objects) {
//     for (const auto& pixel : obj.pixels) {
//       result.push_back(cam_controller_.pixelToCameraCoords(pixel.x, pixel.y, depth_map));
//     }
//   }

//   return result;
// }

void RobotArmNode::printDetectedObjects(const std::shared_ptr<ros_interfaces::srv::GetDetectedObjects::Response>& response)
{
    const auto& class_names = response->class_names;
    const auto& counts = response->counts;
    const auto& pixel_x = response->pixel_x;
    const auto& pixel_y = response->pixel_y;
    const auto& pixel_class_indices = response->pixel_class_indices;

    // index → 구조체
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
  rclcpp::spin(std::make_shared<RobotArmNode>());
  rclcpp::shutdown();
  return 0;
}