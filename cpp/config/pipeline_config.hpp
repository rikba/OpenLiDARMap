#pragma once

#include <cstddef>

namespace openlidarmap::config {

struct PipelineConfig {
    double translation_threshold{};
    double rotation_threshold{};
    size_t sliding_window_size{};
    bool use_sliding_window{};
    bool visualize{};

    PipelineConfig()
        : translation_threshold(0.05),
          rotation_threshold(0.05),
          sliding_window_size(250),
          use_sliding_window(true),
          visualize(true) {}
};

}  // namespace openlidarmap::config
