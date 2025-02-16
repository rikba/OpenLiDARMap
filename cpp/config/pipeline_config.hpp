#pragma once

#include <cstddef>

namespace openlidarmap::config {

struct PipelineConfig {
    double translation_threshold{};
    double rotation_threshold{};
    bool visualize{};

    PipelineConfig() : translation_threshold(0.05), rotation_threshold(0.05), visualize(true) {}
};

}  // namespace openlidarmap::config
