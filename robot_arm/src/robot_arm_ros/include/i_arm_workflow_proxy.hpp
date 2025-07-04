#ifndef I_ARM_WORKFLOW_PROXY_HPP
#define I_ARM_WORKFLOW_PROXY_HPP

#include <future>
#include <string>
#include "ros_interfaces/srv/get_detected_objects.hpp"

class IArmWorkflowProxy {
public:
    virtual ~IArmWorkflowProxy() = default;

    // Service 호출
    virtual std::future<ros_interfaces::srv::GetDetectedObjects::Response::SharedPtr> requestDetections() = 0;

    // Action 피드백 발행
    virtual void publishFeedback(const std::string& status_message) = 0;

    // Action 성공 또는 실패 처리
    virtual void finalizeGoal(bool success, const std::string& final_message) = 0;
    
    // 외부로부터 종료 요청이 있었는지 확인
    virtual bool isCancelRequested() = 0;

    // 로깅
    virtual void logInfo(const std::string& message) = 0;
    virtual void logWarn(const std::string& message) = 0;
    virtual void logError(const std::string& message) = 0;
};

#endif // I_ARM_WORKFLOW_PROXY_HPP 