#pragma once

#include <iostream>

#include "config/ceres_config.hpp"
#include "config/pipeline_config.hpp"
#include "config/preprocess_config.hpp"
#include "config/registration_config.hpp"

namespace openlidarmap::config {

struct Config {
    CeresConfig ceres_{};
    RegistrationConfig registration_{};
    PreProcessConfig preprocess_{};
    PipelineConfig pipeline_{};
};

}  // namespace openlidarmap::config
