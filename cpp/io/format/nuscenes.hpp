#pragma once

#include "io/point_cloud_loader.hpp"

namespace openlidarmap::io {

class NuScenesLoader : public PointCloudLoader {
public:
    explicit NuScenesLoader(config::Config &config) : PointCloudLoader(config) {}
    small_gicp::PointCloud::Ptr load(const std::string &file_path) override;
};

}  // namespace openlidarmap::io