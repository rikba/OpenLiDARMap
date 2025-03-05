#pragma once

#include <memory>

#include "io/point_cloud_loader.hpp"

namespace openlidarmap::io {

enum class FileType { BIN, XYZ, PCD, PLY, NUSCENES, UNKNOWN };

class LoaderFactory {
public:
    static FileType getFileType(const std::string &file_path);
    static std::unique_ptr<PointCloudLoader> createLoader(config::Config &config,
                                                          const std::string &file_path);
    static small_gicp::PointCloud::Ptr loadPointCloud(config::Config &config,
                                                      const std::string &file_path);
};

}  // namespace openlidarmap::io
