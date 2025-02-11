#pragma once

#include <cstddef>

namespace openlidarmap::config {

    struct KernelConfig {
        double model_sse{};
        size_t num_samples{};
        double sigma{};
    
        KernelConfig()
            :  model_sse(1.0),
               num_samples(1),
               sigma(1.0) {}
    };
    
    }  // namespace openlidarmap::config