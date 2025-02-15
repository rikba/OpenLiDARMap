import os
import openlidarmap_pybind as openlidarmap
import numpy as np

# Create test paths
test_data_dir = "/datasets/"

map_path = os.path.join(test_data_dir, "flexblend/kitti/cropped_kitti_map_seq00_origin455_5424_m05.pcd")
scans_dir = os.path.join(test_data_dir, "openlidarmap_plus_plus/data_odometry_velodyne/dataset/sequences/00/velodyne/")
output_path = os.path.join(test_data_dir, "out.txt")

# With custom initial pose
initial_pose = np.array([395.5, 1696.25, 117.55, 0.0, 0.0, 0.4848096, 0.8746197])  # x,y,z, qx,qy,qz,qw
openlidarmap.process_map(
    map_path=map_path,
    scans_dir=scans_dir,
    output_path=output_path,
    initial_pose=initial_pose
)
