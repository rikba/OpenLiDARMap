#pragma once

#include <small_gicp/points/point_cloud.hpp>
#include <string>

#include "config/config.hpp"

namespace openlidarmap::io {

class PointCloudLoader {
public:
    virtual ~PointCloudLoader() = default;
    virtual small_gicp::PointCloud::Ptr load(const std::string &file_path) = 0;

protected:
    explicit PointCloudLoader(config::Config &config) : config_(config) {}
    config::Config &config_;
};

}  // namespace openlidarmap::io
