#pragma once

#include <string>
#include <vector>

#include "config/types.hpp"

namespace openlidarmap::utils {
class FileUtils {
public:
    static std::vector<std::string> getFiles(const std::string &path);
    static void writePosesToCSV(const std::string &filename,
                                const std::vector<Vector7d> &predicted_poses);
};
}  // namespace openlidarmap::utils
