#ifndef ARM_WORKFLOW_MANAGER_HPP
#define ARM_WORKFLOW_MANAGER_HPP

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "domain_types.hpp"
#include "i_arm_workflow_proxy.hpp"
#include "controllers/arm_controller.hpp"
#include "controllers/cam_controller.hpp"
#include "controllers/net_controller.hpp"


class ArmWorkflowManager {
public:
    explicit ArmWorkflowManager(
        IArmWorkflowProxy& ros_proxy,
        std::unique_ptr<ArmController> arm_controller,
        std::unique_ptr<CamController> cam_controller,
        std::unique_ptr<NetController> net_controller);

    void start(const std::vector<arm_core::GoalItem>& goals);
    void stop();
    void tick();

private:
    // State Machine이 가질 수 있는 상태
    enum class State {
        IDLE,
        STARTING_STREAM,
        WAITING_FOR_SERVICE,
        REQUESTING_DETECTION,
        PERFORMING_MANIPULATION,
        RECOVERY,
        FINALIZING_GOAL
    };

    // Camera Frame Streaming 스레드의 활성화 여부
    enum class Activity {
        INACTIVE,
        STREAMING_CAMERA_FRAMES
    };

    void setState(State new_state);
    void setActivity(Activity new_activity);

    void handleState();
    void handleIdle();
    void handleStartingStream();
    void handleWaitingForService();
    void handleRequestingDetection();
    void handlePerformingManipulation();
    void handleRecovery();
    void handleFinalizingGoal();
    
    void startStreamingActivity();
    void stopStreamingActivity();
    void performStreaming();
    bool diagnoseSystem();

    IArmWorkflowProxy& ros_proxy_;
    std::unique_ptr<ArmController> arm_controller_;
    std::unique_ptr<CamController> cam_controller_;
    std::unique_ptr<NetController> net_controller_;
    
    State state_ = State::IDLE;
    Activity activity_ = Activity::INACTIVE;
    
    std::thread activity_thread_;
    std::atomic<bool> stop_activity_flag_{false};

    std::vector<arm_core::DetectedObject> detected_objects_;
    std::vector<arm_core::GoalItem> current_goals_;
    int target_object_index_ = 0;
    int detection_retries_ = 0;
    static const int MAX_DETECTION_RETRIES = 5;
    bool goal_aborted_ = false;
};

#endif