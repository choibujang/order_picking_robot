#include "robot_arm_controllers/cam/cam_controller.hpp"
#include "robot_arm_controllers/net/net_controller.hpp"

int main() {
    CamController cam_controller;
    NetController net_controller;

    cam_controller.startCameraPipeline();
    
    while (true) {
        if (!cam_controller.getFrameSet())
            continue;

        std::vector<uint8_t> data = cam_controller.getColorFrame();
        net_controller.sendMjpegData(data);
    }

}