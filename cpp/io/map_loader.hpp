#pragma once

#include <small_gicp/points/point_cloud.hpp>

namespace openlidarmap::io {

small_gicp::PointCloud::Ptr loadPCD_map(std::string pcd_path);

}  // namespace openlidarmap::io
