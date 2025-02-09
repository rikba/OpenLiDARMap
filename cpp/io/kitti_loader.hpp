#pragma once

#include <small_gicp/points/point_cloud.hpp>
#include <string>

#include "config/config.hpp"

namespace openlidarmap::io {

small_gicp::PointCloud::Ptr loadBIN_kitti(config::Config &config_, std::string bin_path);

}  // namespace openlidarmap::io
