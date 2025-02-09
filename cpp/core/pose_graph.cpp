#include "core/pose_graph.hpp"

#include "core/pose_factor_abs.hpp"
#include "core/pose_factor_rel.hpp"

namespace openlidarmap {

PoseGraph::PoseGraph(const config::Config &config, std::deque<Vector7d> &poses)
    : config_(config), poses_(poses) {
    solver_options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solver_options_.max_num_iterations = config_.ceres_.num_iterations;
    solver_options_.minimizer_progress_to_stdout = config_.ceres_.print;
    solver_options_.num_threads = config_.ceres_.num_threads;

    pose_manifold_ =
        new ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::EigenQuaternionManifold>{};
}

void PoseGraph::addConstraint(size_t idx_from,
                              size_t idx_to,
                              const Vector7d &measurement,
                              bool absolute = false) {
    if (absolute) {
        ceres::ResidualBlockId abs_block_id = problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AbsPoseError, 6, 7>(new AbsPoseError(measurement)),
            new ceres::TukeyLoss(1.0), poses_[idx_to].data());
        abs_residual_blocks_.emplace_back(abs_block_id);
    } else {
        ceres::ResidualBlockId rel_block_id = problem_.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RelPoseError, 6, 7, 7>(new RelPoseError(measurement)),
            new ceres::CauchyLoss(1.0), poses_[idx_from].data(), poses_[idx_to].data());
        rel_residual_blocks_.emplace_back(rel_block_id);

        problem_.SetManifold(poses_[idx_to].data(), pose_manifold_);
    }

    if (poses_.size() == 1) {
        problem_.SetParameterBlockConstant(poses_[0].data());
    }
}

bool PoseGraph::optimize() {
    ceres::Solve(solver_options_, &problem_, &summary_);
    return summary_.IsSolutionUsable();
}

void PoseGraph::manageSlidingWindow() {
    window_indices_.emplace_back(poses_.size() - 1);

    while (window_indices_.size() >= config_.pipeline_.sliding_window_size) {
        window_indices_.pop_front();

        // Remove oldest residual blocks
        if (!abs_residual_blocks_.empty()) {
            problem_.RemoveResidualBlock(abs_residual_blocks_.front());
            abs_residual_blocks_.pop_front();
        }
        if (!rel_residual_blocks_.empty()) {
            problem_.RemoveResidualBlock(rel_residual_blocks_.front());
            rel_residual_blocks_.pop_front();
        }
    }

    // Always keep first pose fixed
    if (!window_indices_.empty()) {
        problem_.SetParameterBlockConstant(poses_[window_indices_.front()].data());
    }
}

}  // namespace openlidarmap
