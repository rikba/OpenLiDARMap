#include "io/format/xyz.hpp"

#include <cstdio>
#include <vector>

namespace openlidarmap::io {

constexpr size_t DEFAULT_BUFFER_SIZE = 1024;

small_gicp::PointCloud::Ptr XYZLoader::load(const std::string &file_path) {
    FILE *file = fopen(file_path.c_str(), "r");
    if (!file) {
        throw std::runtime_error("Cannot open file: " + file_path);
    }

    std::vector<Eigen::Vector4d> pointCloud;
    std::vector<char> line_buffer(DEFAULT_BUFFER_SIZE);
    double x, y, z;

    while (true) {
        if (!fgets(line_buffer.data(), static_cast<int>(line_buffer.size()), file)) {
            if (ferror(file)) {
                fclose(file);
                throw std::runtime_error("Error reading file: " + file_path);
            }
            break;
        }

        // Check for buffer overflow
        if (strlen(line_buffer.data()) == line_buffer.size() - 1) {
            fclose(file);
            throw std::runtime_error("Line too long (max " +
                                     std::to_string(line_buffer.size() - 2) + " chars)");
        }

        if (sscanf(line_buffer.data(), "%lf %lf %lf", &x, &y, &z) == 3) {
            Eigen::Vector3d pt(x, y, z);
            const double norm = pt.norm();
            if (norm > config_.preprocess_.min_range && norm < config_.preprocess_.max_range) {
                pointCloud.emplace_back(x, y, z, 1.0);
            }
        }
    }

    fclose(file);
    return std::make_shared<small_gicp::PointCloud>(pointCloud);
}

}  // namespace openlidarmap::io
