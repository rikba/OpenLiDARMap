#include "core/registration.hpp"

#include <memory>
#include <small_gicp/util/downsampling_tbb.hpp>

namespace openlidarmap {

Registration::Registration(const config::Config &config) : config_(config) {
    configure_registration();
}

void Registration::configure_registration() {
    registration_.rejector.max_dist_sq = config_.registration_.max_correspondence_distance *
                                         config_.registration_.max_correspondence_distance;
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

    target_map_ =
        std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>>(
            config_.registration_.voxel_resolution);
    target_map_->removal_horizon = config_.registration_.removal_horizon;
    target_map_->set_search_offsets(config_.registration_.search_offset);
    target_map_->distance_insert(*map_cloud, initial_guess);

    return true;
}

small_gicp::PointCloud::Ptr Registration::preprocess_cloud(
    const small_gicp::PointCloud::Ptr &cloud) const {
    return small_gicp::voxelgrid_sampling_tbb(*cloud, config_.preprocess_.downsampling_resolution);
}

small_gicp::RegistrationResult Registration::register_frame(
    const small_gicp::PointCloud::Ptr &frame, const Eigen::Isometry3d &initial_guess) {
    auto preprocessed_frame = preprocess_cloud(frame);
    auto result =
        registration_.align(*target_map_, *preprocessed_frame, *target_map_, initial_guess);

    return result;
}

}  // namespace openlidarmap
