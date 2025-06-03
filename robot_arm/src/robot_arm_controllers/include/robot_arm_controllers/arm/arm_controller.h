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

/**
 * @class ArmController
 * @brief URDF 파일로부터 KDL 체인 로드, 목표 위치에 대한 역기구학 계산, 서보 모터 제어 기능 제공
 */
class ArmController {
public:
    ArmController(bool test=false)
    : test_(test),
      pca(test ? PiPCA9685::PCA9685("test", 0x00, true)
               : PiPCA9685::PCA9685())  
    {
        if (!test_) {
            pca_.set_pwm_freq(50);
        }

        std::string urdf_file = ament_index_cpp::get_package_share_directory("my_robot_description") + "/urdf/my_robot.urdf";
        std::string base_link = "base_link";
        std::string end_effector = "link_5";
    
        if (!loadKDLChain(urdf_file, base_link, end_effector)) {
            throw std::runtime_error("Failed to initialize KDL chain.");
        }
    
        // Initialize IK solver
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    
        std::cout << "Robot Arm initialized with KDL chain from URDF." << std::endl;
    }

    /**
     * @brief URDF 파일에서 KDL 체인을 로드한다.
     * 
     * @param urdf_file URDF 파일 경로
     * @param base_link KDL 체인의 시작점이 되는 링크 이름
     * @param end_effector KDL 체인의 끝점이 되는 링크 이름
     * @return true KDL 체인을 성공적으로 로드한 경우
     * @return false KDL 체인을 로드하지 못한 경우
     */
    bool loadKDLChain(const std::string& urdf_file, const std::string& base_link, const std::string& end_effector);
    
    /**
     * @brief 주어진 목표 위치에 대해 역기구학을 계산하여 각 관절의 목표 각도를 반환한다.
     * 
     * @param pick_target_pos 목표 위치의 3D 좌표 (x, y, z)
     * @return std::vector<double> 각 관절의 목표 각도 (라디안 단위)
     */
    std::vector<double> calcIK(std::vector<double> pick_target_pos);

    /**
     * @brief 입력된 각도를 PWM 신호로 변환하여 모터를 제어한다.
     * @param joint_angles 각 관절의 목표 각도 (라디안 단위)
     */
    void moveMotors(const std::vector<double>& joint_angles);

private:
    bool test_;
    PiPCA9685::PCA9685 pca_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::vector<double> current_joints_ = {0, 0, 0, 0, 0, 0};
};
#endif 
