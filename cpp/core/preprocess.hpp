#pragma once

#include "config/config.hpp"
#include <small_gicp/points/point_cloud.hpp>

namespace openlidarmap {

class Preprocess {
public:
    explicit Preprocess(const config::Config& config) : config_(config) {}

    small_gicp::PointCloud::Ptr preprocess_cloud(const small_gicp::PointCloud::Ptr& cloud) const;

private:
    config::Config config_;
};

} // namespace openlidarmap