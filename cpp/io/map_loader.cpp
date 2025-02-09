#include "io/map_loader.hpp"

#include <pcl/io/pcd_io.h>

#include <memory>

#include "config/types.hpp"

namespace openlidarmap::io {

small_gicp::PointCloud::Ptr loadPCD_map(std::string pcd_path) {
    PointCloudT::Ptr cloud(new PointCloudT);
    pcl::io::loadPCDFile<PointT>(pcd_path, *cloud);

    std::vector<Eigen::Vector4d> pointCloud;
    pointCloud.reserve(cloud->points.size());
    for (const auto &point : cloud->points) {
        pointCloud.emplace_back(point.x, point.y, point.z, 1.0);
    }
    return std::make_shared<small_gicp::PointCloud>(pointCloud);
}

}  // namespace openlidarmap::io
