#include "core/registration.hpp"

#include <memory>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>

namespace openlidarmap {

Registration::Registration(const config::Config &config) : config_(config) {
    configure_registration(config_);
}

void Registration::configure_registration(const config::Config &config_) {
    registration_.rejector.max_dist_sq = config_.registration_.max_dist_sq;
    registration_.point_factor.robust_kernel.c = config_.kernel_.sigma;
    registration_.criteria.rotation_eps = config_.registration_.rotation_eps;
    registration_.criteria.translation_eps = config_.registration_.translation_eps;
    registration_.optimizer.max_iterations = config_.registration_.max_iterations;
    registration_.optimizer.verbose = config_.registration_.verbose;
    registration_.optimizer.lambda = config_.registration_.lambda;
}

bool Registration::initialize(const small_gicp::PointCloud::Ptr &map_cloud,
                              const Eigen::Isometry3d &initial_guess) {
    if (!map_cloud || map_cloud->empty()) {
        return false;
    }

    small_gicp::estimate_covariances_tbb(*map_cloud, config_.preprocess_.num_neighbors);

    target_map_ =
        std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>>(
            config_.registration_.voxel_resolution);
    target_map_->voxel_setting.max_num_points_in_cell = config_.registration_.max_num_points_in_cell;
    target_map_->removal_horizon = config_.registration_.removal_horizon;
    target_map_->set_search_offsets(config_.registration_.search_offset);
    target_map_->distance_insert(*map_cloud, initial_guess);

    return true;
}

small_gicp::RegistrationResult Registration::register_frame(
    const small_gicp::PointCloud::Ptr &frame, const Eigen::Isometry3d &initial_guess) {
    auto result =
        registration_.align(*target_map_, *frame, *target_map_, initial_guess);

    return result;
}

void Registration::update_config(const config::Config &config) {
    configure_registration(config);
}
}  // namespace openlidarmap
