#pragma once

#include <ceres/ceres.h>

#include <deque>

#include "config/config.hpp"
#include "config/types.hpp"

namespace openlidarmap {

class PoseGraph {
public:
    explicit PoseGraph(const config::Config &config, std::deque<Vector7d> &poses);

    void addConstraint(size_t idx_from, size_t idx_to, const Vector7d &measurement, bool absolute);
    bool optimize();

private:
    void manageSlidingWindow();

    ceres::Problem problem_{};
    ceres::Solver::Options solver_options_{};
    const config::Config &config_;
    std::deque<Vector7d> &poses_;
    std::deque<ceres::ResidualBlockId> abs_residual_blocks_{};
    std::deque<ceres::ResidualBlockId> rel_residual_blocks_{};
    std::deque<size_t> window_indices_{};
    ceres::Manifold *pose_manifold_{};
    ceres::Solver::Summary summary_{};
};

}  // namespace openlidarmap
