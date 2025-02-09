#pragma once

#include <math.h>

namespace openlidarmap::config {

struct RegistrationConfig {
    double voxel_resolution{};
    double max_correspondence_distance{};
    double rotation_eps{};
    double translation_eps{};
    double lambda{};
    int max_iterations{};
    bool verbose{};
    int map_overlap{};
    int removal_horizon{};
    int search_offset{};
    int max_num_points_in_cell{};

    RegistrationConfig()
        : voxel_resolution(1.0),
          max_correspondence_distance(6.0),
          rotation_eps(0.1 * M_PI / 180.0),
          translation_eps(1e-3),
          lambda(1.0),
          max_iterations(1000),
          verbose(false),
          map_overlap(0),
          removal_horizon(1e9),
          search_offset(27),
          max_num_points_in_cell{100} {}
};

}  // namespace openlidarmap::config
