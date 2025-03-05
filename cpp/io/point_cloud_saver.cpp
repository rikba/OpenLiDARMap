#include "io/point_cloud_saver.hpp"

#include <fstream>
#include <vector>

namespace openlidarmap::io {

void PointCloudSaver::writeHeader(std::ofstream &file, size_t num_points) {
    file << "# .PCD v0.7 - Point Cloud Data\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z\n"
         << "SIZE 4 4 4\n"
         << "TYPE F F F\n"
         << "COUNT 1 1 1\n"
         << "WIDTH " << num_points << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << num_points << "\n"
         << "DATA binary\n";
}

bool PointCloudSaver::savePCD(const std::string &filename, const small_gicp::PointCloud &cloud) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        return false;
    }

    writeHeader(file, cloud.size());

    std::vector<float> buffer(cloud.size() * 3);

    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto &point = cloud.point(i);
        buffer[i * 3] = static_cast<float>(point[0]);
        buffer[i * 3 + 1] = static_cast<float>(point[1]);
        buffer[i * 3 + 2] = static_cast<float>(point[2]);
    }

    file.write(reinterpret_cast<const char *>(buffer.data()), buffer.size() * sizeof(float));

    return file.good();
}

}  // namespace openlidarmap::io
