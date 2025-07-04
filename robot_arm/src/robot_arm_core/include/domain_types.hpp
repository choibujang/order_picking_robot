#ifndef DOMAIN_TYPES_HPP
#define DOMAIN_TYPES_HPP

#include <string>
#include <vector>

namespace arm_core {

// AI 서버에 의해 탐지된 객체 하나를 나타내는 구조체
struct DetectedObject {
    std::string id;
    double x; // 중심점 x
    double y; // 중심점 y
    double score;

    // 테스트 비교를 위한 동등 연산자
    bool operator==(const DetectedObject& other) const {
        return id == other.id && x == other.x && y == other.y && score == other.score;
    }
};
// ROS의 goal 아이템 하나를 나타내는 구조체
struct GoalItem {
    std::string name;
    int quantity;
};

}

#endif 