#include "io/format/ply.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace openlidarmap::io {

bool PLYLoader::ParsePLYHeader(std::ifstream& file, PLYHeader& header) {
    std::string line;
    
    // Check PLY magic
    std::getline(file, line);
    if (line != "ply") return false;

    int vertex_section = 0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;

        if (keyword == "format") {
            std::string format, version;
            iss >> format >> version;
            header.is_binary = (format == "binary_little_endian" || 
                              format == "binary_big_endian");
            header.is_little_endian = (format == "binary_little_endian");
        }
        else if (keyword == "element") {
            std::string type;
            int count;
            iss >> type >> count;
            if (type == "vertex") {
                header.vertex_count = count;
                vertex_section = 1;
            } else {
                vertex_section = 0;
            }
        }
        else if (keyword == "property" && vertex_section) {
            std::string type, name;
            iss >> type >> name;
            header.properties.push_back(name);
            if (name == "x") header.x_idx = header.properties.size() - 1;
            if (name == "y") header.y_idx = header.properties.size() - 1;
            if (name == "z") header.z_idx = header.properties.size() - 1;
        }
        else if (keyword == "end_header") {
            return true;
        }
    }
    return false;
}

inline bool IsLittleEndian() {
    static const uint16_t value = 0x0001;
    return *reinterpret_cast<const uint8_t*>(&value);
}

template<typename T>
inline void SwapEndian(T& value) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    std::reverse(bytes, bytes + sizeof(T));
}

small_gicp::PointCloud::Ptr PLYLoader::load(const std::string& file_path) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open PLY file: " + file_path);
    }

    PLYHeader header;
    if (!ParsePLYHeader(file, header)) {
        throw std::runtime_error("Invalid PLY header in file: " + file_path);
    }

    std::vector<Eigen::Vector4d> points;
    points.reserve(header.vertex_count);
    
    if (header.x_idx < 0 || header.y_idx < 0 || header.z_idx < 0) {
        throw std::runtime_error("Missing XYZ coordinates in PLY file");
    }

    if (header.is_binary) {
        // Read all properties per point
        std::vector<char> point_data(header.properties.size() * sizeof(float));
        
        for (int i = 0; i < header.vertex_count; i++) {
            if (!file.read(point_data.data(), header.properties.size() * sizeof(float))) {
                std::cerr << "Warning: Only read " << i << " of " 
                         << header.vertex_count << " points" << std::endl;
                break;
            }
            
            // Get x,y,z from correct property offsets
            float x = *reinterpret_cast<float*>(point_data.data() + header.x_idx * sizeof(float));
            float y = *reinterpret_cast<float*>(point_data.data() + header.y_idx * sizeof(float));
            float z = *reinterpret_cast<float*>(point_data.data() + header.z_idx * sizeof(float));
            
            if (header.is_little_endian != IsLittleEndian()) {
                SwapEndian(x);
                SwapEndian(y);
                SwapEndian(z);
            }

            const double norm = Eigen::Vector3d(x, y, z).norm();
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                norm > config_.preprocess_.min_range && 
                norm < config_.preprocess_.max_range) {
                points.emplace_back(x, y, z, 1.0);
            }
        }
    } else {
        std::string line;
        std::vector<float> values(header.properties.size());
        
        while (points.size() < header.vertex_count && std::getline(file, line)) {
            std::istringstream iss(line);
            for (float& v : values) {
                iss >> v;
            }

            const double norm = Eigen::Vector3d(values[header.x_idx],
                                              values[header.y_idx],
                                              values[header.z_idx]).norm();
            if (norm > config_.preprocess_.min_range && 
                norm < config_.preprocess_.max_range) {
                points.emplace_back(values[header.x_idx],
                                  values[header.y_idx],
                                  values[header.z_idx], 1.0);
            }
        }
    }

    return std::make_shared<small_gicp::PointCloud>(points);
}

} // namespace openlidarmap::io