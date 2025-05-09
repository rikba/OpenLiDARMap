#include "io/loader_factory.hpp"

#include <filesystem>

#include "io/format/kitti.hpp"
#include "io/format/nuscenes.hpp"
#include "io/format/pcd.hpp"
#include "io/format/ply.hpp"
#include "io/format/xyz.hpp"

namespace openlidarmap::io {

FileType LoaderFactory::getFileType(const std::string &file_path) {
    std::filesystem::path path(file_path);
    std::string ext = path.extension().string();
    std::string filename = path.filename().string();

    if (ext == ".bin" && filename.find(".pcd.bin") != std::string::npos) {
        return FileType::NUSCENES;
    }

    if (ext == ".bin") return FileType::BIN;
    if (ext == ".xyz") return FileType::XYZ;
    if (ext == ".pcd") return FileType::PCD;
    if (ext == ".ply") return FileType::PLY;
    return FileType::UNKNOWN;
}

std::unique_ptr<PointCloudLoader> LoaderFactory::createLoader(config::Config &config,
                                                              const std::string &file_path) {
    auto type = getFileType(file_path);

    switch (type) {
        case FileType::BIN:
            return std::make_unique<KITTILoader>(config);
        case FileType::XYZ:
            return std::make_unique<XYZLoader>(config);
        case FileType::PCD:
            return std::make_unique<PCDLoader>(config);
        case FileType::PLY:
            return std::make_unique<PLYLoader>(config);
        case FileType::NUSCENES:
            return std::make_unique<NuScenesLoader>(config);
        default:
            throw std::runtime_error("Unsupported file type");
    }
}

small_gicp::PointCloud::Ptr LoaderFactory::loadPointCloud(config::Config &config,
                                                          const std::string &file_path) {
    auto loader = createLoader(config, file_path);
    return loader->load(file_path);
}

}  // namespace openlidarmap::io
