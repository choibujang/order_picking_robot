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
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

/**
 * @class ArmController
 * @brief URDF 파일로부터 KDL 체인 로드, 목표 위치에 대한 역기구학 계산, 서보 모터 제어 기능 제공
 */
class ArmController {
public:
    ArmController();

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

    /**
     * @brief 카메라 좌표계 기준 3D 좌표를 월드 좌표계 기준 3D 좌표로 변환한다.
     * @param camera_coords 카메라 좌표계의 3D 좌표 목록
     * @return std::vector<std::vector<float>> 월드 좌표계의 3D 좌표 목록
     */
    std::vector<std::vector<float>> cameraToWorldCoords(const std::vector<std::vector<float>>& camera_coords);

private:
    // 서보 모터의 PWM 신호 범위 (서보 모델에 따라 조정 필요)
    static constexpr int kServoMinPwm = 110;  // 0도에 해당하는 PWM 값
    static constexpr int kServoMaxPwm = 450;  // 180도에 해당하는 PWM 값

    PiPCA9685::PCA9685 pca_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::vector<double> current_joints_ = {0, 0, 0, 0, 0, 0};

    // 월드 좌표계 변환을 위한 카메라의 외부 파라미터 (로봇 베이스 기준)
    // 이 값들은 실제 로봇의 설정에 맞게 보정(calibration)되어야 합니다.
    cv::Mat cam_to_world_rot_ = (cv::Mat_<float>(3, 3) <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1);
    cv::Mat cam_to_world_trans_ = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 0.0); // 예: 로봇 베이스에서 카메라까지의 x, y, z 변위 (미터 단위)
};
#endif 
