#include "core/preprocess.hpp"

#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>

namespace openlidarmap {

small_gicp::PointCloud::Ptr Preprocess::preprocess_cloud(
    const small_gicp::PointCloud::Ptr &cloud) const {
    auto preprocessed_cloud =
        small_gicp::voxelgrid_sampling_tbb(*cloud, config_.preprocess_.downsampling_resolution);

    small_gicp::estimate_covariances_tbb(*preprocessed_cloud, config_.preprocess_.num_neighbors);

    return preprocessed_cloud;
}

}  // namespace openlidarmap
