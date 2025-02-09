#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace openlidarmap {

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

}  // namespace openlidarmap
