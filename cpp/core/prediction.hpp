#pragma once

#include "config/types.hpp"

namespace openlidarmap {

class ConstantDistancePredictor {
public:
    static Vector7d predict(const Vector7d &current_pose, const Vector7d &previous_pose);

private:
    static Eigen::Vector3d predictTranslation(const Vector7d &current, const Vector7d &previous);
    static Eigen::Quaterniond predictRotation(const Vector7d &current, const Vector7d &previous);
    static Eigen::Quaterniond poseToQuaternion(const Vector7d &pose);
};

}  // namespace openlidarmap
