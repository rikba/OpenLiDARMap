#include "utils/helpers.hpp"

#include <iostream>

namespace openlidarmap::utils {

void printProgressBar(size_t current, size_t total) {
    int barWidth = 70;
    float progress = static_cast<float>(current) / total;
    std::cout << '\n' << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

}  // namespace openlidarmap::utils
