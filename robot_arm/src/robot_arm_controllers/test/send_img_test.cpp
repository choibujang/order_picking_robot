#include "robot_arm_controllers/cam/cam_controller.hpp"
#include "robot_arm_controllers/net/net_controller.hpp"

int main() {
    CamController cam_controller;
    NetController net_controller(1, "", 8080);

    cam_controller.start();
    
    while (true) {
        if (!cam_controller.update())
            continue;

        std::vector<uint8_t> data = cam_controller.getColorFrame();
        net_controller.sendMjpegData(data);
    }

}