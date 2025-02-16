#pragma once
#include <string>
#include <vector>

#include "io/point_cloud_loader.hpp"

namespace openlidarmap::io {

struct PLYHeader {
    bool is_binary = false;
    bool is_little_endian = true;
    int vertex_count = 0;
    std::vector<std::string> properties;
    int x_idx = -1, y_idx = -1, z_idx = -1;
};

class PLYLoader : public PointCloudLoader {
public:
    explicit PLYLoader(config::Config &config) : PointCloudLoader(config) {}
    small_gicp::PointCloud::Ptr load(const std::string &file_path) override;

private:
    bool ParsePLYHeader(std::ifstream &file, PLYHeader &header);
};

}  // namespace openlidarmap::io
