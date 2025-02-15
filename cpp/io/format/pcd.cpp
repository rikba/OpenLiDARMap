#include "io/format/pcd.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <iomanip>


namespace openlidarmap::io {

struct PCDField {
    std::string name;
    int size = 4;      // default size for float
    char type = 'F';   // F(float), I(signed), U(unsigned)
    int count = 1;     // number of elements
    int offset = 0;    // byte offset in point data
};

struct PCDHeader {
    std::string version;
    std::vector<PCDField> fields;
    int width = 0;
    int height = 0;
    int points = 0;
    bool is_binary = false;
    bool is_compressed = false;
    int point_size = 0;
};

bool ParsePCDHeader(std::ifstream& file, PCDHeader& header) {
    std::string line;
    int offset = 0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;

        if (keyword == "VERSION") {
            iss >> header.version;
        } else if (keyword == "FIELDS") {
            std::string field_name;
            while (iss >> field_name) {
                PCDField field;
                field.name = field_name;
                header.fields.push_back(field);
            }
        } else if (keyword == "SIZE") {
            for (size_t i = 0; i < header.fields.size(); i++) {
                iss >> header.fields[i].size;
            }
        } else if (keyword == "TYPE") {
            for (size_t i = 0; i < header.fields.size(); i++) {
                iss >> header.fields[i].type;
            }
        } else if (keyword == "COUNT") {
            for (size_t i = 0; i < header.fields.size(); i++) {
                iss >> header.fields[i].count;
                header.fields[i].offset = offset;
                offset += header.fields[i].size * header.fields[i].count;
            }
        } else if (keyword == "WIDTH") {
            iss >> header.width;
        } else if (keyword == "HEIGHT") {
            iss >> header.height;
        } else if (keyword == "POINTS") {
            iss >> header.points;
        } else if (keyword == "DATA") {
            std::string format;
            iss >> format;
            header.is_binary = (format == "binary");
            header.is_compressed = (format == "binary_compressed");
            break;
        }
    }

    header.point_size = offset;
    return true;
}

double UnpackBinary(const char* data_ptr, char type, int size) {
    switch (type) {
        case 'F': {
            if (size == 4) {
                float value;
                std::memcpy(&value, data_ptr, sizeof(float));
                return static_cast<double>(value);
            } else if (size == 8) {
                double value;
                std::memcpy(&value, data_ptr, sizeof(double));
                return value;
            }
            break;
        }
        case 'I': {
            switch (size) {
                case 1: return static_cast<double>(*reinterpret_cast<const int8_t*>(data_ptr));
                case 2: return static_cast<double>(*reinterpret_cast<const int16_t*>(data_ptr));
                case 4: return static_cast<double>(*reinterpret_cast<const int32_t*>(data_ptr));
                case 8: return static_cast<double>(*reinterpret_cast<const int64_t*>(data_ptr));
            }
            break;
        }
        case 'U': {
            switch (size) {
                case 1: return static_cast<double>(*reinterpret_cast<const uint8_t*>(data_ptr));
                case 2: return static_cast<double>(*reinterpret_cast<const uint16_t*>(data_ptr));
                case 4: return static_cast<double>(*reinterpret_cast<const uint32_t*>(data_ptr));
                case 8: return static_cast<double>(*reinterpret_cast<const uint64_t*>(data_ptr));
            }
            break;
        }
    }
    throw std::runtime_error("Unsupported PCD data type or size");
}

small_gicp::PointCloud::Ptr PCDLoader::load(const std::string& file_path) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file: " + file_path);
    }

    PCDHeader header;
    if (!ParsePCDHeader(file, header)) {
        throw std::runtime_error("Invalid PCD header in file: " + file_path);
    }

    int x_idx = -1, y_idx = -1, z_idx = -1;
    for (size_t i = 0; i < header.fields.size(); i++) {
        if (header.fields[i].name == "x") x_idx = i;
        if (header.fields[i].name == "y") y_idx = i;
        if (header.fields[i].name == "z") z_idx = i;
    }

    std::vector<Eigen::Vector4d> points;
    points.reserve(header.points);

    if (header.is_binary) {
        if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
            throw std::runtime_error("Missing XYZ fields in PCD file");
        }

        // Reset file position and search for DATA line
        file.clear();
        file.seekg(0);
        
        std::string line;
        bool found_data = false;
        while (std::getline(file, line)) {
            if (line.find("DATA binary") != std::string::npos) {
                found_data = true;
                break;
            }
        }

        if (!found_data) {
            throw std::runtime_error("Failed to find binary data section");
        }

        std::vector<char> point_data(header.point_size);
        int points_read = 0;

        // Rest of binary reading remains the same...
        for (int i = 0; i < header.points; i++) {
            if (!file.read(point_data.data(), header.point_size)) {
                std::cerr << "Warning: Only read " << points_read << " of " 
                         << header.points << " points" << std::endl;
                break;
            }
            
            float x, y, z;
            std::memcpy(&x, point_data.data() + header.fields[x_idx].offset, sizeof(float));
            std::memcpy(&y, point_data.data() + header.fields[y_idx].offset, sizeof(float));
            std::memcpy(&z, point_data.data() + header.fields[z_idx].offset, sizeof(float));

            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                const double norm = Eigen::Vector3d(x, y, z).norm();
                if (norm > config_.preprocess_.min_range && 
                    norm < config_.preprocess_.max_range && 
                    z > -5.0) {
                    points.emplace_back(x, y, z, 1.0);
                }
            }
            points_read++;
        }
    } else {
        std::string line;
        while (std::getline(file, line) && points.size() < header.points) {
            std::istringstream iss(line);
            std::vector<double> values(header.fields.size());
            
            for (size_t i = 0; i < header.fields.size(); i++) {
                iss >> values[i];
            }

            const double x = values[x_idx];
            const double y = values[y_idx];
            const double z = values[z_idx];
            
            const double norm = Eigen::Vector3d(x, y, z).norm();
            if (norm > config_.preprocess_.min_range && 
                norm < config_.preprocess_.max_range && 
                z > -5.0) {
                points.emplace_back(x, y, z, 1.0);
            }
        }
    }

    return std::make_shared<small_gicp::PointCloud>(points);
}

} // namespace openlidarmap::io