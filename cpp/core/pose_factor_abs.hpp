#pragma once

#include "config/types.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace openlidarmap {
struct AbsPoseError {
    AbsPoseError(const Vector7d &abs_measure) : abs_measure_(abs_measure) {}

    template <typename T>
    bool operator()(const T *const pose, T *residual) const {
        constexpr double STDDEV = 1.0;
        
        for (int i = 0; i < 3; ++i) {
            residual[i] = (pose[i] - T(abs_measure_[i])) / T(STDDEV);
        }

        // Quaternion error
        Eigen::Quaternion<T> q1(pose[6], pose[3], pose[4], pose[5]);
        Eigen::Quaternion<T> q2{T(abs_measure_[6]), T(abs_measure_[3]), 
                                T(abs_measure_[4]), T(abs_measure_[5])};
        Eigen::Quaternion<T> q_error = q1.normalized() * q2.normalized().inverse();

        // Convert quaternion error to residual
        residual[3] = T(2.0) * q_error.x() / T(STDDEV);
        residual[4] = T(2.0) * q_error.y() / T(STDDEV);
        residual[5] = T(2.0) * q_error.z() / T(STDDEV);

        return true;
    }

private:
    const Vector7d abs_measure_;
};
}  // namespace openlidarmap
