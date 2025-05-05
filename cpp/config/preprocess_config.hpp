#pragma once

namespace openlidarmap::config {

struct PreProcessConfig {
    double min_range{};
    double max_range{};
    double downsampling_resolution{};
    int num_neighbors{};

    PreProcessConfig()
        : min_range(1.0), max_range(100.0), downsampling_resolution(0.0), num_neighbors(10) {}
};

}  // namespace openlidarmap::config
