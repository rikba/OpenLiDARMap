#pragma once

#include <ceres/ceres.h>

namespace openlidarmap::config {

struct CeresConfig {
    ceres::LinearSolverType solver_type{};
    int num_iterations{};
    bool print{};
    int num_threads{};

    CeresConfig()
        : solver_type(ceres::SPARSE_NORMAL_CHOLESKY),
          num_iterations(100),
          print(false),
          num_threads(12) {}
};

}  // namespace openlidarmap::config
