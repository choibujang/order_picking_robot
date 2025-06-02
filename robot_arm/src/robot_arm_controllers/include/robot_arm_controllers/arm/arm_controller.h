/***********************************
카메라 기준 물체 좌표를 입력받아
- 로봇 기준 좌표계로 변환 
- 목표 위치까지의 경로 계산
- 모터 제어
기능을 하는 클래스
************************************/

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <cmath>
#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>

#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "pca9685_comm.h"

class ArmController {
public:
    ArmController(bool test=false)
    : test_(test),
      pca(test ? PiPCA9685::PCA9685("test", 0x00, true)
               : PiPCA9685::PCA9685())  
    {
        if (!test_) {
            pca.set_pwm_freq(50);
        }

        std::string urdf_file = ament_index_cpp::get_package_share_directory("my_robot_description") + "/urdf/my_robot.urdf";
        std::string base_link = "base_link";
        std::string end_effector = "link_5";
    
        if (!loadKDLChain(urdf_file, base_link, end_effector)) {
            throw std::runtime_error("Failed to initialize KDL chain.");
        }
    
        // Initialize IK solver
        ik_solver = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain);
    
        std::cout << "Robot Arm initialized with KDL chain from URDF." << std::endl;
    }

    bool loadKDLChain(const std::string& urdf_file, const std::string& base_link, const std::string& end_effector);
    std::vector<double> calcIK(std::vector<double> pick_target_pos);

private:
    bool test_;
    PiPCA9685::PCA9685 pca;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
    std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};
};
#endif 
