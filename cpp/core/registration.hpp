#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/optimizer.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <small_gicp/registration/rejector.hpp>

#include "config/config.hpp"

namespace openlidarmap {

class Registration {
public:
    explicit Registration(const config::Config &config);

    bool initialize(const small_gicp::PointCloud::Ptr &map_cloud,
                    const Eigen::Isometry3d &initial_guess);

    small_gicp::RegistrationResult register_frame(const small_gicp::PointCloud::Ptr &frame,
                                                  const Eigen::Isometry3d &initial_guess);

    const small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>::Ptr &get_map() const {
        return target_map_;
    }

private:
    using RegistrationType = small_gicp::Registration<
        small_gicp::RobustFactor<small_gicp::GemanMcClure, small_gicp::ICPFactor>,
        small_gicp::ParallelReductionTBB,
        small_gicp::NullFactor,
        small_gicp::DistanceRejector,
        small_gicp::GaussNewtonOptimizer>;

    void configure_registration();
    small_gicp::PointCloud::Ptr preprocess_cloud(const small_gicp::PointCloud::Ptr &cloud) const;

    config::Config config_{};
    RegistrationType registration_{};
    small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerPoints>::Ptr target_map_{};
};

}  // namespace openlidarmap
