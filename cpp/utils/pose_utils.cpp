#include "utils/pose_utils.hpp"

#include <iomanip>
#include <sstream>

namespace openlidarmap::utils {

Vector7d PoseUtils::isometryToPoseVector(const Eigen::Isometry3d &isometry) {
    Vector7d pose_vec{};
    pose_vec.head<3>() = isometry.translation();
    Eigen::Quaterniond q(isometry.linear());
    q.normalize();
    pose_vec[3] = q.x();
    pose_vec[4] = q.y();
    pose_vec[5] = q.z();
    pose_vec[6] = q.w();

    return pose_vec;
}

Eigen::Isometry3d PoseUtils::poseVectorToIsometry(const Vector7d &pose_vec) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5]);
    q.normalize();
    isometry.linear() = q.toRotationMatrix();
    isometry.translation() = pose_vec.head<3>();

    return isometry;
}

bool PoseUtils::isMoving(const Eigen::Isometry3d &T_target_source,
                         const Vector7d &predicted_pose,
                         double trans_thresh,
                         double rot_thresh) {
    Eigen::Vector3d translation = T_target_source.translation();
    Eigen::Quaterniond rotation(T_target_source.rotation());

    Eigen::Vector3d predicted_translation(predicted_pose[0], predicted_pose[1], predicted_pose[2]);
    Eigen::Quaterniond predicted_rotation(predicted_pose[6], predicted_pose[3], predicted_pose[4],
                                          predicted_pose[5]);

    double translation_diff = (translation - predicted_translation).norm();
    double rotation_diff = rotation.angularDistance(predicted_rotation);

    return translation_diff >= trans_thresh || rotation_diff >= rot_thresh;
}

std::string PoseUtils::convertMatrixToKitti(const Eigen::Matrix4d &matrix) {
    std::ostringstream oss;
    oss << std::setprecision(18) << std::scientific;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            oss << matrix(i, j);
            if (!(i == 2 && j == 3)) {
                oss << " ";
            }
        }
    }
    return oss.str();
}

}  // namespace openlidarmap::utils
