#include "robot_arm_controllers/arm/arm_controller.h"

bool ArmController::loadKDLChain(const std::string& urdf_file, const std::string& base_link, const std::string& end_effector) {
    std::ifstream urdf_stream(urdf_file);
    if (!urdf_stream.is_open()) {
        std::cerr << "Failed to open URDF file: " << urdf_file << std::endl;
        return false;
    }

    std::string urdf_content((std::istreambuf_iterator<char>(urdf_stream)), std::istreambuf_iterator<char>());
    urdf_stream.close();

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(urdf_content, kdl_tree)) {
        std::cerr << "Failed to parse URDF into KDL tree." << std::endl;
        return false;
    }

    if (!kdl_tree.getChain(base_link, end_effector, kdl_chain_)) {
        std::cerr << "Failed to extract KDL chain from base_link to end_effector." << std::endl;
        return false;
    }

    std::cout << "Successfully loaded KDL chain from URDF." << std::endl;
    return true;
}

std::vector<double> ArmController::calcIK(std::vector<double> pick_target_pos) {
    if (pick_target_pos.size() != 3) {
        throw std::invalid_argument("Target position must have 3 elements (x, y, z).");
    }

    KDL::Frame target_frame(KDL::Vector(pick_target_pos[0], pick_target_pos[1], pick_target_pos[2]));
    KDL::JntArray q_init(kdl_chain_.getNrOfJoints());
    KDL::JntArray q_result(kdl_chain_.getNrOfJoints());

    for (size_t i = 0; i < current_joints_.size(); ++i) {
        q_init(i) = current_joints_[i] * M_PI / 180.0;
    }

    int result = ik_solver->CartToJnt(q_init, target_frame, q_result);
    if (result < 0) {
        std::cerr << "IK solver failed with error code: " << result << std::endl;
        return std::vector<double>(current_joints.size(), std::nan(""));
    }

    std::vector<double> joint_angles(q_result.rows());
    for (size_t i = 0; i < joint_angles.size(); ++i) {
        joint_angles[i] = q_result(i) * 180.0 / M_PI;
    }

    return joint_angles;
}

void ArmController::moveMotors(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != current_joints_.size()) {
        throw std::invalid_argument("Size of joint_angles does not match the number of joints.");
    }

    for (size_t i = 0; i < joint_angles.size(); ++i) {
        double angle_deg = joint_angles[i] * (180.0 / M_PI);
        int pwm_value = static_cast<int>(500 + (angle_deg / 180.0) * 2000);

        pca_.set_pwm(i, 0, pwm_value);

        current_joints_[i] = joint_angles[i];

        std::cout << "Joint " << i << " moved to angle: " << joint_angles[i] << " rad (" << angle_deg << " deg)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}