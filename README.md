<div align="center">
    <h1>OpenLiDARMap</h1>
    <h2>Zero-Drift Point Cloud Mapping using Map Priors</h2>
  <br>

  [![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
  ![License](https://img.shields.io/badge/license-Apache%202.0-blue)
  ![Version](https://img.shields.io/badge/version-0.1.0-blue)
  [![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/2501.11111)
  
  <br align="center">
  
  [![openlidarmap](doc/openlidarmap_seq00.gif)](https://youtu.be/JqTDPS63eIE)
  <br>

</div>

## Concept

We use reference maps in combination with classical LiDAR odometry to enable drift-free localization/mapping. Our approach is designed for high precision mapping. It enables georeferenced LiDAR-only point cloud mapping without GNSS. A detailed description of our pipeline can be found in the linked paper.

<img src=doc/pipeline_diagram.png alt="diagram" width="480" />

## Usage

<details>
<summary>Install</summary>

We provide a Docker image on Docker Hub, which will automatically be pulled within the Run section, but you also have the option to build it locally.  
```sh
./docker/build_docker.sh # (optional)
```
</details>

<details>
<summary>Run</summary>

To use our approach, you need a reference map and an initial guess of the first pose.

The easiest way to use our approach is with the provided Docker image.
```sh
./docker/run_docker.sh <map_path> <scan_path> <output_path> <x> <y> <z> <qx> <qy> <qz> <qw>

# Example
./docker/run_docker.sh datasets/kitti/map.pcd datasets/data_odometry_velodyne/dataset/sequences/00/velodyne output.txt 395.5 1696.25 117.55 0 0 0.4848096 0.8746197
```

The output of the algorithm are poses in the KITTI format.
  
We also provide Python bindings. Have a look in the `python` folder, where we provide a test script.

</details>
<details>
<summary>Configure</summary>

The configuration of this pipeline can be changed in the `cpp/config` files. The naming suggest the intended usecase for the files. The most important parameters to play with if your results are not as good as expected are:

| Parameter | Description | Default | Note |
| :-------- | :-------- | :--------: | :-------- |
| pipeline_.visualize | Toggle GUI | `true` | use `false` on headless servers |
| preprocess_.downsampling_resolution | Scans are voxelized before usage | `1.5` | Reduce the size for increased robustness |
| preprocess_.num_neighbors | Points for covariance calculation | `10` | Try both directions |
| registration_.voxel_resolution | Voxelhashmap voxel size | `1.0` | Reduce the size for increased robustness | 
| registration_.lambda | Optimization dampening factor | `1.0` | Increase to increase the robustness |


</details>
<details>
<summary>Develop</summary>

We also provida a Development image, if you like to contribute or adapt or approach.  
Open this repository in VSCode -> F1 -> Rebuild and Reopen in Container.  

To build the C++ code:
```sh
mkdir build
cd build
cmake ../cpp && make -j
```

To build the Python bindings:
```sh
cd python
pip install -e .
```

</details>


## Limitations

* Currently only the KITTI .bin dataloader is implemented
* The reference map has to be in the .pcd format
* Detailed instructions on how to create refrence maps is missing
* Currently the visualization is active on default
* Move to nanobind

## Acknowledgement

Great inspiration has come from the following repositories. If you use our work, please also leave a star in their repositories and cite their work.

* [KISS-ICP](https://github.com/PRBonn/kiss-icp)
* [small_gicp](https://github.com/koide3/small_gicp)
* [Iridescence](https://github.com/koide3/iridescence)


## Citation

```bibtex
@article{kulmer2025openlidarmap,
  author    = {Kulmer, Dominik and Leitenstern, Maximilian and Weinmann, Marcel and Lienkamp, Markus},
  title     = {{OpenLiDARMap: Zero-Drift Point Cloud Mapping using Map Priors}},
  pages     = {1-11},
  doi       = {10.48550/arXiv.2501.11111},
  year      = {2025},
  url       = {https://arxiv.org/abs/2501.11111},
  codeurl   = {https://github.com/TUMFTM/OpenLiDARMap},
}
```
