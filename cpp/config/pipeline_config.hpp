#pragma once

#include <cstddef>

namespace openlidarmap::config {

struct PipelineConfig {
    double translation_threshold{};
    double rotation_threshold{};
    size_t sliding_window_size{};
    bool use_sliding_window{};

    PipelineConfig()
        : translation_threshold(0.1),
          rotation_threshold(0.1),
          sliding_window_size(250),
          use_sliding_window(true) {}
};

}  // namespace openlidarmap::config
