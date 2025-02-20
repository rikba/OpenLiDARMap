#pragma once
#include <small_gicp/points/point_cloud.hpp>
#include <string>

namespace openlidarmap::io {

class PointCloudSaver {
public:
    static bool savePCD(const std::string& filename, const small_gicp::PointCloud& cloud);
private:
    static void writeHeader(std::ofstream& file, size_t num_points);
};

}  // namespace openlidarmap::io