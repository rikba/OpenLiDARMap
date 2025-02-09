#include "utils/file_utils.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "utils/pose_utils.hpp"

namespace openlidarmap::utils {

std::vector<std::string> FileUtils::getFiles(const std::string &path) {
    std::vector<std::string> file_paths;
    for (const auto &file_path : std::filesystem::recursive_directory_iterator(path)) {
        file_paths.emplace_back(file_path.path());
    }
    std::sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

void FileUtils::writePosesToCSV(const std::string &filename,
                                const std::vector<Vector7d> &predicted_poses) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file: " << filename << std::endl;
        return;
    }

    file << std::setprecision(18) << std::scientific;
    for (const auto &pose : predicted_poses) {
        file << PoseUtils::convertMatrixToKitti(PoseUtils::poseVectorToIsometry(pose).matrix())
             << '\n';
    }
}

}  // namespace openlidarmap::utils
