#include "io/kitti_loader.hpp"

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <memory>

namespace openlidarmap::io {

small_gicp::PointCloud::Ptr loadBIN_kitti(config::Config &config_, std::string bin_path) {
    std::ifstream file(bin_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + bin_path);
    }

    // Get the file size
    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read the file into a buffer
    std::vector<float> buffer(size / sizeof(float));
    if (!file.read(reinterpret_cast<char *>(buffer.data()), size)) {
        throw std::runtime_error("Error reading file: " + bin_path);
    }
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    std::vector<Eigen::Vector4d> pointCloud;
    pointCloud.reserve(buffer.size() / 4);
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < buffer.size(); i += 4) {
        Eigen::Vector3d pt{buffer[i], buffer[i + 1], buffer[i + 2]};
        const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
        pt = Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
        const double norm = pt.norm();
        if (norm > config_.preprocess_.min_range && norm < config_.preprocess_.max_range &&
            pt[2] > -5.0) {
            pointCloud.emplace_back(pt[0], pt[1], pt[2], 1.0);
        }
    }

    return std::make_shared<small_gicp::PointCloud>(pointCloud);
}

}  // namespace openlidarmap::io
