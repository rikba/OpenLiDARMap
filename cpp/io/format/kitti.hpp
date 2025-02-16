#pragma once

#include "io/point_cloud_loader.hpp"

namespace openlidarmap::io {

class KITTILoader : public PointCloudLoader {
public:
    explicit KITTILoader(config::Config &config) : PointCloudLoader(config) {}
    small_gicp::PointCloud::Ptr load(const std::string &file_path) override;
};

}  // namespace openlidarmap::io
