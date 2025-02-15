#pragma once

#include "io/point_cloud_loader.hpp"

namespace openlidarmap::io {

class PCDLoader : public PointCloudLoader {
public:
    explicit PCDLoader(config::Config& config) : PointCloudLoader(config) {}
    small_gicp::PointCloud::Ptr load(const std::string& file_path) override;
};

}  // namespace openlidarmap::io