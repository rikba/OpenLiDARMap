#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "config/types.hpp"

namespace openlidarmap::utils {

class PoseUtils {
public:
    static Vector7d isometryToPoseVector(const Eigen::Isometry3d &isometry);
    static Eigen::Isometry3d poseVectorToIsometry(const Vector7d &pose_vec);
    static bool isMoving(const Eigen::Isometry3d &current,
                         const Vector7d &predicted,
                         double trans_thresh = 0.1,
                         double rot_thresh = 0.1);
    static std::string convertMatrixToKitti(const Eigen::Matrix4d &matrix);
};

}  // namespace openlidarmap::utils
