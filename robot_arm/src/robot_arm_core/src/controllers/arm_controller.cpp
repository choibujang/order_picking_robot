#include "controllers/arm_controller.h"


ArmController::ArmController(): pca_(PiPCA9685::PCA9685()) {
  pca_.set_pwm_freq(50);

  std::string urdf_file =
      ament_index_cpp::get_package_share_directory("my_robot_description") +
      "/urdf/my_robot.urdf";
  std::string base_link = "base_link";
  std::string end_effector = "link_5";

  if (!loadKDLChain(urdf_file, base_link, end_effector)) {
    throw std::runtime_error("Failed to initialize KDL chain.");
  }

  // Initialize IK solver
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);

  std::cout << "Robot Arm initialized with KDL chain from URDF." << std::endl;
}

bool ArmController::loadKDLChain(const std::string& urdf_file,
                                 const std::string& base_link,
                                 const std::string& end_effector) {
  // URDF 파일에서 KDL 트리 생성
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_file, tree)) {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return false;
  }

  // 트리에서 base_link부터 end_effector까지의 체인 추출
  if (!tree.getChain(base_link, end_effector, kdl_chain_)) {
    std::cerr << "Failed to get KDL chain from tree" << std::endl;
    return false;
  }
  return true;
}

std::vector<double> ArmController::calcIK(
    std::vector<double> pick_target_pos) {
  // 초기 관절 각도 (현재 각도 또는 기본값)
  KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    q_init(i) = current_joints_[i];
  }

  // 목표 위치 설정
  KDL::Frame target_frame = KDL::Frame(
      KDL::Vector(pick_target_pos[0], pick_target_pos[1], pick_target_pos[2]));

  // IK 솔버 실행
  KDL::JntArray q_result(kdl_chain_.getNrOfJoints());
  int ret = ik_solver_->CartToJnt(q_init, target_frame, q_result);

  // 결과 저장
  std::vector<double> result_angles;
  if (ret >= 0) {
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
      result_angles.push_back(q_result(i));
    }
  }

  return result_angles;
}

void ArmController::moveMotors(const std::vector<double>& joint_angles) {
  for (size_t i = 0; i < joint_angles.size(); ++i) {
    // 라디안을 도로 변환
    double angle_deg = joint_angles[i] * 180.0 / M_PI;
    // -90 ~ 90 범위를 0 ~ 180으로 변환
    angle_deg += 90;

    // 0 ~ 180도 범위를 PWM 값으로 변환
    int pwm_value = static_cast<int>(
        kServoMinPwm + (angle_deg / 180.0) * (kServoMaxPwm - kServoMinPwm));
    pca_.set_pwm(i, 0, pwm_value);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));  // 모터가 움직일 시간을 줌
  }
}

std::vector<std::vector<float>> ArmController::cameraToWorldCoords(const std::vector<std::vector<float>>& camera_coords) {
    std::vector<std::vector<float>> world_coords;
    world_coords.reserve(camera_coords.size());

    for (const auto& p_cam_vec : camera_coords) {
        cv::Mat p_cam = (cv::Mat_<float>(3, 1) << p_cam_vec[0], p_cam_vec[1], p_cam_vec[2]);
        cv::Mat p_world = cam_to_world_rot_ * p_cam + cam_to_world_trans_;

        world_coords.push_back({p_world.at<float>(0), p_world.at<float>(1), p_world.at<float>(2)});
    }
    
    return world_coords;
}