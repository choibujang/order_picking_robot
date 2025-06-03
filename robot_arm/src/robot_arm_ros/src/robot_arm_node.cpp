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

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<DispatchManipulationTask::Feedback>();
  auto result = std::make_shared<DispatchManipulationTask::Result>();

  // =============================
  // 1. AI 서버로 영상 전송 스레드 시작
  // =============================
  startSendFrameThread();
  RCLCPP_INFO(this->get_logger(), "Start Send Frame Thread");

  // ====================================================
  // 2. AI 서버의 get_detected_object 서비스 대기 (재시도 3회)
  // ====================================================
  int wait_for_service_retries = 3;

  while (wait_for_service_retries-- > 0 && !service_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        stopSendFrameThread();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for service to appear... (%d retries left)", wait_for_service_retries);
  }
  
  // AI 서버의 서비스를 찾지 못한 경우 goal Abort
  if (wait_for_service_retries < 0) {
    stopSendFrameThread();
    result->success = false;
    result->error_code = 2;
    result->error_msg = "Cannot find /get_detected_objects service."
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Goal aborted: Cannot find AI server's service.");
    return;
  }

  // AI 서버의 서비스를 찾은 경우
  feedback->progress = "Requesting Service to AI Server."
  goal_handle->publish_feedback(feedback);


  // ==============================================
  // 3. AI 서버에 객체 검출 요청 (재시도 3회) -> 응답 검사
  // ==============================================
  int service_request_retries = 3;
  bool service_request_success = false;
  std::vector<std::pair<int,int>>> pixel_coords;

  enum class ErrorType {NONE, PERIPHERAL, INSUFFICIENT, TIMEOUT} last_error = ErrorType::NONE;
  
  for (int attempt = 0; attempt < service_request_retries; ++attempt) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while requesting for the service.");
      stopSendFrameThread();
      return;
    }
    // 3.1 요청 생성 및 비동기 전송, 최대 5초 대기
    auto request = std::make_shared<GetDetectedObjects::Request>();
    auto future = service_client_->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));
  
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      // 3.2 응답이 온 경우
      auto response = future.get();
      printDetectedObjects(response);
  
      // goal에 명시된 물체들이 충분히 검출되었는지 확인
      pixel_coords = checkDetectionSatisfied(goal->item_names, goal->item_quantities, response);
      if (!pixel_coords.empty()) {
        // 충분한 경우
        service_request_success = true;
        break;
      } else {
        // 충분하지 않은 경우
        RCLCPP_WARN(this->get_logger(), "Detection not satisfied. Attempt %d/%d", attempt + 1, service_request_retries);
        
        // 카메라 또는 네트워크 에러 확인
        if (net_controller_.isNetworkError() || cam_controller_.isCameraError()) {
          last_error = ErrorType::PERIPHERAL;
          RCLCPP_INFO(this->get_logger(), "Detected network or camera error. Restarting components.");
  
          // 영상 전송 스레드 중지 후 잠시 대기
          stopSendFrameThread();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
          // 네트워크/카메라 컨트롤러 다시 초기화
          net_controller_ = NetController(device_id_, server_ip_, server_port_);
          cam_controller_ = CamController();
  
          // 영상 전송 스레드 재시작
          StartSendFrameThread();
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
          // 물체 종류나 개수가 부족하게 검출된 상태
          last_error = ErrorType::INSUFFICIENT;
        }
      }
    } else if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
      // 3.3 서비스 응답 타임아웃
      last_error = ErrorType::TIMEOUT;
      RCLCPP_WARN(this->get_logger(), "Timed out waiting for /get_detected_objects service response. Attempt %d/%d", attempt + 1, service_request_retries);
    }
  }
  
  // =======================
  // 4. 서비스 요청-응답 후 처리
  // =======================
  
  stopSendFrameThread();

  if (!service_request_success) {
    // AI 서버로부터 적절한 응답을 받지 못한 경우
    RCLCPP_ERROR(this->get_logger(), "Failed to satisfy detection condition after %d attempts.", service_request_retries);

    result->success = false;
    if (last_error == ErrorType::PERIPHERAL) {
      result->error_code = 1;  // 1: 주변 장치 에러
      result->error_msg = "Peripheral error detected during object detection.";
      RCLCPP_ERROR(this->get_logger(), "Goal aborted: Peripheral error.");
    } else if (last_error == ErrorType::INSUFFICIENT) {
      result->error_code = 3;  // 3: 검출 개수 부족
      result->error_msg = "Detected objects are fewer than requested quantities.";
      RCLCPP_ERROR(this->get_logger(), "Goal aborted: Insufficient detected objects.");
    } else if (last_error == ErrorType::TIMEOUT) {
      result->error_code = 4;  // 4: AI 서버 timeout
      result->error_msg = "No response from AI Server.";
      RCLCPP_ERROR(this->get_logger(), "Goal aborted: AI Server timeout.");    
    } 
    goal_handle->abort(result);
    return;
  }

  // AI 서버로부터 적절한 답을 받은 경우
  feedback->progress = "Received valid response from the AI Server."
  goal_handle->publish_feedback(feedback);

  
  // =================================
  // 5. 픽셀 좌표 -> 카메라 기준 좌표로 변환
  // =================================
  auto camera_coords = cam_controller_.pixelToCameraCoords(pixel_coords);

  
  
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

std::vector<std::pair<int, int>>> RobotArmNode::checkDetectionSatisfied(
  const std::vector<std::string>& item_names,
  const std::vector<int>& item_quantities,
  const std::shared_ptr<ros_interfaces::srv::GetDetectedObjects::Response>& response) 
{
  // 결과를 담을 벡터
  std::vector<std::pair<int, int>>> result;

  const auto& class_names = response->class_names;
  const auto& counts = response->counts;
  const auto& pixel_x = response->pixel_x;
  const auto& pixel_y = response->pixel_y;
  const auto& pixel_class_indices = response->pixel_class_indices;

  // goal의 item_names를 순회하면서
  for (size_t i = 0; i < item_names.size(); ++i) {
      const std::string& target_name = item_names[i];
      int required_quantity = item_quantities[i];

      // 1. 해당 item이 class_names에 있는지 확인
      auto it = std::find(class_names.begin(), class_names.end(), target_name);
      if (it == class_names.end()) {
          RCLCPP_INFO(rclcpp::get_logger("check"), "Item %s not detected.", target_name.c_str());
          // 없으면 빈 맵 반환
          return {};
      }

      // 2. counts[class_idx]이 required_quantity 이상인지 확인
      size_t class_idx = std::distance(class_names.begin(), it);
      int detected_quantity = counts[class_idx];

      if (detected_quantity < required_quantity) {
          RCLCPP_INFO(rclcpp::get_logger("check"), "Item %s detected only %d, but required %d.",
                      target_name.c_str(), detected_quantity, required_quantity);
          // 적으면 빈 맵 반환
          return {};
      }

      // 3. class_idx에 해당하는 픽셀 좌표를 required_quantity만큼 수집
      int stored = 0;
      for (size_t pix_i = 0; pix_i < pixel_class_indices.size(); ++pix_i) {
        if (pixel_class_indices[pix_i] == class_idx) {
          result.push_back({pixel_x[pix_i], pixel_y[pix_i]});
          ++stored;

          if (stored == required_quantity) {
            break;
          }
        }
      }
  }
  
  return result;
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