#include "io/format/nuscenes.hpp"

#include <fstream>
#include <vector>

namespace openlidarmap::io {

small_gicp::PointCloud::Ptr NuScenesLoader::load(const std::string &file_path) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + file_path);
    }

    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    // NuScenes format has 5 floats per point: x, y, z, intensity, ring
    const size_t point_size = 5 * sizeof(float);
    if (size % point_size != 0) {
        throw std::runtime_error("Invalid nuScenes point cloud file size");
    }

    const size_t num_points = size / point_size;

    std::vector<float> buffer(size / sizeof(float));
    if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
        throw std::runtime_error("Error reading file: " + file_path);
    }

    std::vector<Eigen::Vector4d> pointCloud;
    pointCloud.reserve(num_points);

    for (size_t i = 0; i < buffer.size(); i += 5) {
        const float x = buffer[i];
        const float y = buffer[i + 1];
        const float z = buffer[i + 2];
        // float intensity = buffer[i + 3];
        // float ring = buffer[i + 4];

        const double norm = Eigen::Vector3d(x, y, z).norm();
        if (norm > config_.preprocess_.min_range && norm < config_.preprocess_.max_range) {
            pointCloud.emplace_back(x, y, z, 1.0);
        }
    }

    return std::make_shared<small_gicp::PointCloud>(pointCloud);
}

}  // namespace openlidarmap::io