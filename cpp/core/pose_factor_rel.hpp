#pragma once

#include "config/types.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace openlidarmap {
struct RelPoseError {
    RelPoseError(const Vector7d &rel_measure) : rel_measure_(rel_measure) {}

    template <typename T>
    bool operator()(const T *const pose_i, const T *const pose_j, T *residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos_i(pose_i);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos_j(pose_j);

        // Extract quaternions (x,y,z,w order) to Eigen (w,x,y,z order)
        Eigen::Quaternion<T> q_i(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        Eigen::Quaternion<T> q_j(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);

        // Convert relative measurement
        Eigen::Matrix<T, 3, 1> rel_pos{T(rel_measure_[0]), T(rel_measure_[1]), T(rel_measure_[2])};
        Eigen::Quaternion<T> rel_q{T(rel_measure_[6]), T(rel_measure_[3]), T(rel_measure_[4]),
                                   T(rel_measure_[5])};

        // Position error
        residual[0] = pos_j.x() - rel_pos.x();
        residual[1] = pos_j.y() - rel_pos.y();
        residual[2] = pos_j.z() - rel_pos.z();

        // Quaternion error
        Eigen::Quaternion<T> q_error = q_j.normalized() * rel_q.normalized().conjugate();

        residual[3] = T(2.0) * q_error.x();
        residual[4] = T(2.0) * q_error.y();
        residual[5] = T(2.0) * q_error.z();

        return true;
    }

private:
    const Vector7d rel_measure_;
};
}  // namespace openlidarmap
