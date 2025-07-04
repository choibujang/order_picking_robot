#include "arm_workflow_manager.hpp"


using namespace std::chrono_literals;

ArmWorkflowManager::ArmWorkflowManager(
    IArmWorkflowProxy& ros_proxy,
    std::unique_ptr<ArmController> arm_controller,
    std::unique_ptr<CamController> cam_controller,
    std::unique_ptr<NetController> net_controller)
    : ros_proxy_(ros_proxy),
      arm_controller_(std::move(arm_controller)),
      cam_controller_(std::move(cam_controller)),
      net_controller_(std::move(net_controller)) {
    ros_proxy_.logInfo("ArmWorkflowManager initialized.");
}

void ArmWorkflowManager::start(const std::vector<arm_core::GoalItem>& goals) {
    if (state_ == State::IDLE) {
        ros_proxy_.logInfo("Workflow started.");
        setState(State::STARTING_STREAM);
        goal_aborted_ = false;
        target_object_index_ = 0;
        detection_retries_ = 0;
        detected_objects_.clear();
        current_goals_ = goals;
    } else {
        ros_proxy_.logWarn("Cannot start workflow: not in IDLE state.");
    }
}

void ArmWorkflowManager::stop() {
    ros_proxy_.logInfo("Workflow stop requested.");
    setState(State::IDLE);
    stopStreamingActivity();
}

void ArmWorkflowManager::tick() {
    if (ros_proxy_.isCancelRequested() && !goal_aborted_) {
        ros_proxy_.logInfo("Goal cancellation requested by client.");
        ros_proxy_.finalizeGoal(false, "Goal cancelled by client.");
        goal_aborted_ = true;
        setState(State::IDLE);
        return;
    }

    if (goal_aborted_) {
        return;
    }
    
    handleState();
}

void ArmWorkflowManager::setState(State new_state) {
    if (state_ != new_state) {
        state_ = new_state;
    }
}

void ArmWorkflowManager::setActivity(Activity new_activity) {
    if (activity_ != new_activity) {
        activity_ = new_activity;
    }
}

void ArmWorkflowManager::handleState() {
    switch (state_) {
        case State::IDLE:                  handleIdle();                  break;
        case State::STARTING_STREAM:       handleStartingStream();       break;
        case State::WAITING_FOR_SERVICE:   handleWaitingForService();   break;
        case State::REQUESTING_DETECTION:  handleRequestingDetection();  break;
        case State::PERFORMING_MANIPULATION: handlePerformingManipulation(); break;
        case State::RECOVERY:              handleRecovery();              break;
        case State::FINALIZING_GOAL:       handleFinalizingGoal();       break;
    }
}

void ArmWorkflowManager::handleIdle() {
    // RobotArmNode에 goal이 도착해 start()가 호출되기를 기다린다.
}

void ArmWorkflowManager::handleStartingStream() {
    ros_proxy_.logInfo("[State] STARTING_STREAM");
    ros_proxy_.publishFeedback("Starting camera stream...");
    startStreamingActivity();
    if (activity_ == Activity::STREAMING_CAMERA_FRAMES) {
        setState(State::WAITING_FOR_SERVICE);
    } else {
        ros_proxy_.logError("Failed to start streaming activity. Entering recovery.");
        setState(State::RECOVERY);
    }
}

void ArmWorkflowManager::handleWaitingForService() {
    ros_proxy_.logInfo("[State] WAITING_FOR_SERVICE");
    ros_proxy_.publishFeedback("Waiting for detection service...");

    setState(State::REQUESTING_DETECTION);
}

void ArmWorkflowManager::handleRequestingDetection() {
    ros_proxy_.logInfo("[State] REQUESTING_DETECTION (Attempt " + std::to_string(detection_retries_ + 1) + ")");
    ros_proxy_.publishFeedback("Requesting object detections...");

    if (!detection_future_.valid()) {
        detection_future_ = ros_proxy_.requestDetections();
    }

    auto future_status = detection_future_.wait_for(1s);

    if (future_status == std::future_status::ready) {
        try {
            auto response = detection_future_.get();
            if (response && !response->detections.empty()) {
                ros_proxy_.logInfo("Detections received!");
                detected_objects_ = response->detections;
                setState(State::PERFORMING_MANIPULATION);
            } else {
                ros_proxy_.logWarn("Detection returned no objects. Retrying...");
                detection_retries_++;
                if (detection_retries_ >= MAX_DETECTION_RETRIES) {
                    ros_proxy_.finalizeGoal(false, "Failed to detect any objects after all retries.");
                    goal_aborted_ = true;
                    setState(State::IDLE);
                }
                detection_future_ = {}; 
            }
        } catch (const std::exception& e) {
             ros_proxy_.logError("Exception while getting detection future: " + std::string(e.what()));
             setState(State::RECOVERY);
        }
        detection_future_ = {}; 
    } else if (future_status == std::future_status::timeout) {
        ros_proxy_.logWarn("Detection service timed out. Checking system health...");
        if (!diagnoseSystem()) {
            ros_proxy_.logError("System diagnosis failed. Entering recovery.");
            setState(State::RECOVERY);
        } else {
             ros_proxy_.logWarn("System is healthy. Assuming transient issue. Retrying...");
             detection_retries_++;
             if (detection_retries_ >= MAX_DETECTION_RETRIES) {
                ros_proxy_.finalizeGoal(false, "Detection service timed out after all retries.");
                goal_aborted_ = true;
                setState(State::IDLE);
             }
             detection_future_ = {}; 
        }
    }
}

void ArmWorkflowManager::handlePerformingManipulation() {
    ros_proxy_.logInfo("[State] PERFORMING_MANIPULATION");
    ros_proxy_.publishFeedback("Stopping stream and performing manipulation...");
    stopStreamingActivity();

    if (target_object_index_ < detected_objects_.size()) {
        auto& object = detected_objects_[target_object_index_];
        try {
            ros_proxy_.logInfo("Processing object: " + object.id);
            std::this_thread::sleep_for(2s); 
            ros_proxy_.publishFeedback("Successfully manipulated object " + object.id);
            target_object_index_++;

            if (target_object_index_ >= detected_objects_.size()) {
                setState(State::FINALIZING_GOAL);
            }
        } catch (const std::runtime_error& e) {
            ros_proxy_.logError("Failed to manipulate object: " + std::string(e.what()));
            ros_proxy_.finalizeGoal(false, "A critical error occurred during manipulation.");
            goal_aborted_ = true;
            setState(State::IDLE);
        }
    } else {
        ros_proxy_.logInfo("All detected objects have been processed.");
        setState(State::FINALIZING_GOAL);
    }
}

void ArmWorkflowManager::handleRecovery() {
    ros_proxy_.logError("[State] RECOVERY");
    ros_proxy_.publishFeedback("System error detected. Attempting recovery...");
    
    stopStreamingActivity();
    
    ros_proxy_.logInfo("Cooldown before restarting activity...");
    std::this_thread::sleep_for(2s);

    ros_proxy_.logInfo("Restarting stream activity...");
    setState(State::STARTING_STREAM);
}

void ArmWorkflowManager::handleFinalizingGoal() {
    ros_proxy_.logInfo("[State] FINALIZING_GOAL");
    ros_proxy_.publishFeedback("Workflow completed successfully.");
    stopStreamingActivity();
    ros_proxy_.finalizeGoal(true, "All objects processed successfully.");
    goal_aborted_ = true;
    setState(State::IDLE);
}

void ArmWorkflowManager::startStreamingActivity() {
    if (activity_ == Activity::STREAMING_CAMERA_FRAMES) {
        ros_proxy_.logWarn("Streaming activity is already running.");
        return;
    }
    stop_activity_flag_.store(false);
    activity_thread_ = std::thread(&ArmWorkflowManager::performStreaming, this);
    setActivity(Activity::STREAMING_CAMERA_FRAMES);
    ros_proxy_.logInfo("Streaming activity started.");
}

void ArmWorkflowManager::stopStreamingActivity() {
    if (activity_ == Activity::INACTIVE) {
        return;
    }
    stop_activity_flag_.store(true);
    if (activity_thread_.joinable()) {
        activity_thread_.join();
    }
    setActivity(Activity::INACTIVE);
    ros_proxy_.logInfo("Streaming activity stopped.");
}

void ArmWorkflowManager::performStreaming() {
    try {
        while (!stop_activity_flag_.load()) {
            auto frame = cam_controller_->getFrame();
            if (!frame.empty()) {
                net_controller_->sendFrame(frame);
            }
            std::this_thread::sleep_for(33ms);
        }
    } catch (const std::runtime_error& e) {
        ros_proxy_.logError("Exception in streaming thread: " + std::string(e.what()));
        setActivity(Activity::INACTIVE);
    }
}

bool ArmWorkflowManager::diagnoseSystem() {
    bool is_healthy = true;
    if (activity_ != Activity::STREAMING_CAMERA_FRAMES) {
        ros_proxy_.logError("Diagnosis: Streaming activity is not running.");
        is_healthy = false;
    }
    
    return is_healthy;
} 