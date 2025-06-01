#include "robot_arm_controllers/cam/cam_controller.hpp"
#include <atomic>

int main() {
    int cnt = 0;
    CamController cam_controller;

    if (!cam_controller.start()) {
        std::cerr << "Camera failed to start.\n";
        return -1;
    }

    while (cnt < 10) {
        if (!cam_controller.update()) {
            continue;
        }

        cv::Mat current_depth_frame = cam_controller.getDepthFrame();
        float value = current_depth_frame.at<float>(200, 320);
        std::cout << value << "mm" << std::endl;
        cnt++;
    }

    cam_controller.stop();
    cv::Mat current_depth_map = cam_controller.getDepthMap();
    float value = current_depth_map.at<float>(240, 320);
    std::cout << value << "mm" << std::endl;

    return 0;
}