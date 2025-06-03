#ifndef CAM_CONTROLLER_HPP
#define CAM_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "libobsensor/ObSensor.hpp"

/**
 * @class CamController
 * @brief 카메라 on/off, 프레임 가져오기, depth map 생성, 픽셀 좌표->3d좌표 변환 기능 제공
 */
class CamController {
public:
    CamController() {}

    /**
     * @brief 카메라의 color, depth 스트리밍을 활성화한다.
     */
    bool start();

    /**
     * @brief 활성화된 스트림에서 frameset을 가져와 클래스 멤버 current_color_frame_, color_depth_frame_에 저장한다.
     * @return frameset 수신 성공 여부
     */
    bool update();

    /**
     * @brief 카메라 스트리밍을 중지한다.
     */
    void stop();

    /**
     * @brief RGB 이미지의 특정 픽셀 좌표를 카메라 좌표계 기준 3D 좌표로 변환한다.
     * @param pixel_coords RGB 이미지의 x, y좌표
     * @return 해당 픽셀의 카메라 기준 3d 좌표
     */
    std::vector<std::vector<float>> pixelToCameraCoords(const std::vector<std::pair<int,int>& pixel_coords);

    /**
     * @brief 카메라의 intrinsic parameter들을 출력한다.
     */ 
    void getCameraParam();

    std::vector<uint8_t>  getColorFrame() {
        std::vector<uint8_t> color_frame;
        {
            std::lock_guard<std::mutex> lock(color_frame_mutex_);
            color_frame = current_color_frame_;
        }
        return color_frame;
    }

    cv::Mat getDepthFrame() {
        cv::Mat depth_frame;
        {
            std::lock_guard<std::mutex> lock(depth_frame_mutex_);
            depth_frame = current_depth_frame_;
        }
        return depth_frame;
    }

    cv::Mat getDepthMap() {
        updateDepthMap();

        return current_depth_map_.clone();
    }

    bool isCameraError() {
        return camera_error_;
    }

private:
    /**
     * @brief 640*400 current_depth_frame_을 기반으로 
     *        640*480 RGB 카메라 기준으로 정렬된 Depth Map을 생성.
     *        RGB 카메라와 Depth 카메라의 해상도 차이로 인해
     *        Depth = 0인 빈 영역은 3x3 주변 평균으로 보간하여 채움.
     */
    void updateDepthMap();
    
    
    ob::Pipeline pipe_;
    std::vector<uint8_t> current_color_frame_;
    cv::Mat current_depth_frame_;
    cv::Mat current_depth_map_;
    std::mutex color_frame_mutex_;
    std::mutex depth_frame_mutex_;

    bool camera_error_ = false;

    float depth_fx_ = 475.328;
    float depth_fy_ = 475.328;
    float depth_cx_ = 315.204;
    float depth_cy_ = 196.601;
    float depth_width_ = 640;
    float depth_height_ = 400;

    float rgb_fx_ = 453.183;
    float rgb_fy_ = 453.183;
    float rgb_cx_ = 333.191;
    float rgb_cy_ = 241.26;
    float rgb_width_ = 640;
    float rgb_height_ = 480;

    // RGB to Depth
    cv::Mat rgb_to_depth_rot_ = (cv::Mat_<float>(3, 3) << 
        0.999983, 0.0050659, -0.0028331,
        -0.0050672, 0.999987,-0.00045272,
        0.00283077, 0.000467068, 0.999996
    );

    // Translation Vector
    cv::Mat rgb_to_depth_trans_ = (cv::Mat_<float>(3, 1) << 9.98615, -0.0882425, 0.675267);

    cv::Mat depth_to_rgb_rot_ = (cv::Mat_<float>(3, 3) <<
        0.999983, -0.0050672, 0.00283077,
        0.0050659, 0.999987, 0.000467068,
        -0.0028331, -0.00045272, 0.999996
    );

    cv::Mat depth_to_rgb_trans_ = (cv::Mat_<float>(3, 1) << -9.98834, 0.0373371, -0.647012);


};

#endif
