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
        constexpr double STDDEV = 1.0;

        Eigen::Transform<T,3,Eigen::Isometry> T_i = Eigen::Transform<T,3,Eigen::Isometry>::Identity();
        T_i.translation() = Eigen::Map<const Eigen::Matrix<T,3,1>>(pose_i);
        Eigen::Quaternion<T> q_i(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        T_i.linear() = q_i.normalized().toRotationMatrix();

        Eigen::Transform<T,3,Eigen::Isometry> T_j = Eigen::Transform<T,3,Eigen::Isometry>::Identity();
        T_j.translation() = Eigen::Map<const Eigen::Matrix<T,3,1>>(pose_j);
        Eigen::Quaternion<T> q_j(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);
        T_j.linear() = q_j.normalized().toRotationMatrix();

        Eigen::Transform<T,3,Eigen::Isometry> T_target = Eigen::Transform<T,3,Eigen::Isometry>::Identity();
        T_target.translation() = Eigen::Matrix<T,3,1>(
            T(rel_measure_[0]),
            T(rel_measure_[1]), 
            T(rel_measure_[2])
        );

        const T w = T(rel_measure_[6]);
        const T x = T(rel_measure_[3]);
        const T y = T(rel_measure_[4]);
        const T z = T(rel_measure_[5]);
        Eigen::Quaternion<T> q_target(w, x, y, z);
        T_target.linear() = q_target.normalized().toRotationMatrix();

        auto T_error = T_target.inverse() * T_j;

        Eigen::Matrix<T,3,1> pos_error = T_error.translation();
        
        Eigen::Quaternion<T> q_error(T_error.linear());
        q_error.normalize();

        residual[0] = pos_error.x() / T(STDDEV);
        residual[1] = pos_error.y() / T(STDDEV);
        residual[2] = pos_error.z() / T(STDDEV);
        
        residual[3] = T(2.0) * q_error.x() / T(STDDEV);
        residual[4] = T(2.0) * q_error.y() / T(STDDEV); 
        residual[5] = T(2.0) * q_error.z() / T(STDDEV);

        return true;
    }

private:
    const Vector7d rel_measure_;
};
}  // namespace openlidarmap
