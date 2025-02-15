#include "utils/helpers.hpp"

#include <iomanip>
#include <iostream>

namespace openlidarmap::utils {

void printProgressBar(float progress, double processing_time) {
    int barWidth = 50;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << "%";
    std::cout << " (Processing Time: " << std::fixed << std::setprecision(2) << processing_time
              << " ms)\r";
    std::cout.flush();
}

}  // namespace openlidarmap::utils
