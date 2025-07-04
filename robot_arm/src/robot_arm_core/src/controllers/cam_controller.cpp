#include "controllers/cam_controller.hpp"


CamController::CamController() {
    // Constructor is now empty
}

CamController::~CamController() {
    try {
        stop();
    } catch (...) {
        // Destructor should not throw. Log or ignore.
    }
}

void CamController::start()
{
    try {
        // 기존의 device list, profile list를 찾고 config를 설정하는 로직은 그대로 유지
        auto dev_list = pipe_.getDeviceList();
        if(dev_list->deviceCount() == 0) {
            throw std::runtime_error("No Orbbec device found!");
        }
        auto dev = dev_list->getDevice(0);
        
        auto config = std::make_shared<ob::Config>();

        auto color_profile_list = pipe_.getStreamProfileList(OB_SENSOR_COLOR);
        if(!color_profile_list || color_profile_list->count() == 0) {
            throw std::runtime_error("No color stream profile found!");
        }
        auto color_profile = color_profile_list->getVideoStreamProfile(640, 0, OB_FORMAT_MJPG, 30);
        if(!color_profile) {
            color_profile = color_profile_list->getProfile(0);
        }
        config->enableStream(color_profile);

        auto depth_profile_list = pipe_.getStreamProfileList(OB_SENSOR_DEPTH);
        if(!depth_profile_list || depth_profile_list->count() == 0) {
            throw std::runtime_error("No depth stream profile found!");
        }
        auto depth_profile = depth_profile_list->getVideoStreamProfile(640, 0, OB_FORMAT_Y16, 30);
        if(!depth_profile) {
            depth_profile = depth_profile_list->getProfile(0);
        }
        config->enableStream(depth_profile);
        
        config->setAlignMode(ALIGN_D2C_HW_MODE);

        // 파이프라인 시작
        pipe_.start(config, [this](std::shared_ptr<ob::FrameSet> frame_set){
            // ... 기존 프레임 처리 로직 ...
        });
        
    } catch (const ob::Error& e) {
        throw std::runtime_error(std::string("Failed to start camera: ") + e.getMessage());
    }
}

void CamController::update()
{
    int update_failures = 0;

    while (update_failures < kMaxUpdateFailures) {
        try {
            auto frame_set = pipe_.waitForFrames(kFrameWaitTimeoutMs);
            if(frame_set == nullptr) {
                update_failures++;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue; // Retry
            }

            if (auto color_frame = frame_set->colorFrame()) {
              std::lock_guard<std::mutex> lock(color_frame_mutex_);
              auto* frame_data = reinterpret_cast<const uint8_t*>(color_frame->data());
              current_color_frame_.assign(frame_data, frame_data + color_frame->dataSize());
            } else {
                throw std::runtime_error("Color frame is null.");
            }

            if (auto depth_frame = frame_set->depthFrame()) {
              std::lock_guard<std::mutex> lock(depth_frame_mutex_);
              current_depth_frame_ = cv::Mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data()).clone();
            } else {
                throw std::runtime_error("Depth frame is null.");
            }
            
            // Success
            return;

        } catch (const ob::Error& e) {
            update_failures++;
            if (update_failures >= kMaxUpdateFailures) {
                 throw std::runtime_error(std::string("Failed to update frame after multiple retries: ") + e.getMessage());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    // Loop exited due to max retries for waitForFrames timeout
    throw std::runtime_error("Timeout waiting for frameset after multiple retries.");
}

void CamController::stop()
{
    try {
        pipe_.stop();
    } catch(const ob::Error& e) {
        // stop에서 발생하는 에러는 throw하지 않고 콘솔에만 출력
        std::cerr << "Error during pipe stop: " << e.getMessage() << std::endl;
    }
}

void CamController::updateDepthMap() {
  cv::Mat depth_frame;
  {
    std::lock_guard<std::mutex> lock(depth_frame_mutex_);
    if (current_depth_frame_.empty()) {
      return;  // 원본 뎁스 프레임이 없으면 종료
    }
    depth_frame = current_depth_frame_.clone();
  }

  cv::Mat depth_map(rgb_height_, rgb_width_, CV_16UC1, cv::Scalar(0));

  // 뎁스 카메라의 각 픽셀을 순회
  for (int y = 0; y < depth_height_; y++) {
    for (int x = 0; x < depth_width_; x++) {
      uint16_t depth_value = depth_frame.at<uint16_t>(y, x);
      if (depth_value == 0) continue;

      // 뎁스 카메라 픽셀 좌표 (x, y, depth_value)를 3D 포인트로 변환
      float Z_d = depth_value;
      float X_d = (x - depth_cx_) * Z_d / depth_fx_;
      float Y_d = (y - depth_cy_) * Z_d / depth_fy_;
      
      cv::Mat P_d = (cv::Mat_<float>(3, 1) << X_d, Y_d, Z_d);

      // 뎁스 3D 포인트를 RGB 카메라 좌표계로 변환
      cv::Mat P_c = depth_to_rgb_rot_ * P_d + depth_to_rgb_trans_;
      float X_c = P_c.at<float>(0, 0);
      float Y_c = P_c.at<float>(1, 0);
      float Z_c = P_c.at<float>(2, 0);

      // RGB 카메라 좌표계의 3D 포인트를 RGB 이미지 픽셀 좌표로 투영
      int u = static_cast<int>(rgb_fx_ * X_c / Z_c + rgb_cx_);
      int v = static_cast<int>(rgb_fy_ * Y_c / Z_c + rgb_cy_);

      // 픽셀 좌표가 RGB 이미지 범위 내에 있으면 뎁스 값을 맵에 기록
      if (u >= 0 && u < rgb_width_ && v >= 0 && v < rgb_height_) {
        depth_map.at<uint16_t>(v, u) = Z_c * 1000.0f; // m to mm
      }
    }
  }
  
  // 뎁스맵의 빈 공간(0 값)을 주변 픽셀 평균으로 보간
  cv::Mat temp_map = depth_map.clone();
  for (int v = 1; v < rgb_height_ - 1; ++v) {
      for (int u = 1; u < rgb_width_ - 1; ++u) {
          if (temp_map.at<uint16_t>(v, u) == 0) {
              int sum = 0;
              int count = 0;
              // 3x3 주변 픽셀 확인
              for (int j = -1; j <= 1; ++j) {
                  for (int i = -1; i <= 1; ++i) {
                      if (i == 0 && j == 0) continue;
                      uint16_t val = temp_map.at<uint16_t>(v + j, u + i);
                      if (val > 0) {
                          sum += val;
                          count++;
                      }
                  }
              }
              // 유효한 주변 픽셀이 있으면 평균값으로 채움
              if (count > 0) {
                  depth_map.at<uint16_t>(v, u) = sum / count;
              }
          }
      }
  }
  
  // 완성된 뎁스맵 저장
  current_depth_map_ = depth_map;
}

void CamController::getCameraParam() {
  try {
    // 카메라 파라미터 가져오기
    auto param = pipe_.getCameraParam();
    // 뎁스 카메라 내장 파라미터 출력
    std::cout << "Depth Intrinsic: fx=" << param.depthIntrinsic.fx
              << " fy=" << param.depthIntrinsic.fy
              << " cx=" << param.depthIntrinsic.cx
              << " cy=" << param.depthIntrinsic.cy
              << " width=" << param.depthIntrinsic.width
              << " height=" << param.depthIntrinsic.height << std::endl;

    // RGB 카메라 내장 파라미터 출력
    std::cout << "RGB Intrinsic: fx=" << param.rgbIntrinsic.fx
              << " fy=" << param.rgbIntrinsic.fy
              << " cx=" <<param.rgbIntrinsic.cx
              << " cy=" << param.rgbIntrinsic.cy
              << " width=" << param.rgbIntrinsic.width
              << " height=" << param.rgbIntrinsic.height << std::endl;

    // 두 카메라 간의 외장 파라미터(회전 및 변환) 출력
    std::cout << "Rotation from RGB to Depth:" << std::endl;
    for (int i = 0; i < 9; ++i) {
        std::cout << param.transform.rot[i] << (i % 3 == 2 ? "\n" : " ");
    }
    std::cout << "Translation from RGB to Depth:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << param.transform.trans[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "  - p1, p2: " << rgb_p1 << ", " << rgb_p2 << std::endl;

  } catch (const ob::Error& e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
              << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
  }
}


std::vector<std::vector<float>> CamController::pixelToCameraCoords(const std::vector<std::pair<int,int>>& pixel_coords)
{
    updateDepthMap();

    std::vector<std::vector<float>> camera_coords;
    camera_coords.reserve(pixel_coords.size());

    for (const auto& p : pixel_coords) {
        int u = p.first;
        int v = p.second;

        if (u < 0 || u >= current_depth_map_.cols || v < 0 || v >= current_depth_map_.rows) {
            throw std::out_of_range("Pixel coordinates are out of bounds.");
        }

        float Z = current_depth_map_.at<uint16_t>(v, u) / 1000.0f; // mm to m

        if (Z <= 0) {
            throw std::runtime_error("Invalid depth value for pixel coordinate transformation.");
        }
        
        float X = (u - rgb_cx_) * Z / rgb_fx_;
        float Y = (v - rgb_cy_) * Z / rgb_fy_;
        camera_coords.push_back({X, Y, Z});
    }
    return camera_coords;
}
