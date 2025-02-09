#include "core/prediction.hpp"

namespace openlidarmap {

Vector7d ConstantDistancePredictor::predict(const Vector7d &current_pose,
                                            const Vector7d &previous_pose) {
    Vector7d predicted_pose;

    // Predict translation
    predicted_pose.head<3>() = predictTranslation(current_pose, previous_pose);

    // Predict rotation
    Eigen::Quaterniond q_next = predictRotation(current_pose, previous_pose);

    // Pack quaternion into pose vector
    predicted_pose[3] = q_next.x();
    predicted_pose[4] = q_next.y();
    predicted_pose[5] = q_next.z();
    predicted_pose[6] = q_next.w();

    return predicted_pose;
}

Eigen::Vector3d ConstantDistancePredictor::predictTranslation(const Vector7d &current,
                                                              const Vector7d &previous) {
    const Eigen::Vector3d velocity = current.head<3>() - previous.head<3>();
    return current.head<3>() + velocity;
}

Eigen::Quaterniond ConstantDistancePredictor::predictRotation(const Vector7d &current,
                                                              const Vector7d &previous) {
    Eigen::Quaterniond q_current = poseToQuaternion(current);
    Eigen::Quaterniond q_previous = poseToQuaternion(previous);

    Eigen::Quaterniond q_next = q_previous.slerp(2.0, q_current);
    q_next.normalize();

    return q_next;
}

Eigen::Quaterniond ConstantDistancePredictor::poseToQuaternion(const Vector7d &pose) {
    return Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
}

}  // namespace openlidarmap
