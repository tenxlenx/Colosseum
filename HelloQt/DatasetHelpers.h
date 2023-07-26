#pragma once
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <common/Common.hpp>
#include "gpu_dct.cuh"
#include "sh_transform.cuh"
#include <vector>
#include <algorithm>
#include <utility>

/*
class DataSet
{
public:
    void read_dataset(std::string& path)
    {
        DataSet dataset;

        std::ifstream input_file(path);
        std::string line;
        while (std::getline(input_file, line)) {
            std::istringstream line_stream(line);
            std::string timestamp_str, x_str, y_str, z_str, yaw_str, pitch_str, roll_str,
                vel_x_str, vel_y_str, vel_z_str, ang_vel_x_str, ang_vel_y_str,
                ang_vel_z_str, image_name;
            std::getline(line_stream, timestamp_str, ',');
            std::getline(line_stream, x_str, ',');
            std::getline(line_stream, y_str, ',');
            std::getline(line_stream, z_str, ',');
            std::getline(line_stream, yaw_str, ',');
            std::getline(line_stream, pitch_str, ',');
            std::getline(line_stream, roll_str, ',');
            std::getline(line_stream, vel_x_str, ',');
            std::getline(line_stream, vel_y_str, ',');
            std::getline(line_stream, vel_z_str, ',');
            std::getline(line_stream, ang_vel_x_str, ',');
            std::getline(line_stream, ang_vel_y_str, ',');
            std::getline(line_stream, ang_vel_z_str, ',');
            std::getline(line_stream, image_name);

            long timestamp = std::stol(timestamp_str);
            float x = std::stof(x_str);
            float y = std::stof(y_str);
            float z = std::stof(z_str);
            float yaw = std::stof(yaw_str);
            float pitch = std::stof(pitch_str);
            float roll = std::stof(roll_str);
            float vel_x = std::stof(vel_x_str);
            float vel_y = std::stof(vel_y_str);
            float vel_z = std::stof(vel_z_str);
            float ang_vel_x = std::stof(ang_vel_x_str);
            float ang_vel_y = std::stof(ang_vel_y_str);
            float ang_vel_z = std::stof(ang_vel_z_str);
        }
        (timestamp, x, y, z, yaw, pitch, roll, vel_x, vel_y, vel_z, ang_vel_x, ang_vel_y, ang_vel_z, image_name);

        dataset.add_data_point(data);
    }

    std::vector<std::bitset<64>> read_bitsets_from_file(const std::string& filename)
    {
        std::vector<std::bitset<64>> bitsets;

        std::ifstream file(filename, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error opening file for reading: " << filename << std::endl;
            return bitsets;
        }

        while (true) {
            uint64_t value;
            file.read(reinterpret_cast<char*>(&value), sizeof(uint64_t));
            if (file.eof()) {
                break;
            }
            bitsets.push_back(std::bitset<64>(value));
        }

        file.close();
        return bitsets;
    }


    void write_bitsets_to_file(const std::string& filename, const std::vector<std::bitset<64>>& bitsets)
    {
        std::ofstream file(filename, std::ios::out | std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return;
        }

        for (const auto& bitset : bitsets) {
            uint64_t value = bitset.to_ulong();
            file.write(reinterpret_cast<const char*>(&value), sizeof(uint64_t));
        }

        file.close();
    }
    

    void add_data_point(const DataPoint& data_point)
    {
        data_.push_back(data_point);
    }

    
    float distance(int index1, int index2) const
    {
        const DataPoint& point1 = data_[index1];
        const DataPoint& point2 = data_[index2];
        float dx = point2.get_x() - point1.get_x();
        float dy = point2.get_y() - point1.get_y();
        float dz = point2.get_z() - point1.get_z();
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    float angular_distance(int index1, int index2) const
    {
        const DataPoint& point1 = data_[index1];
        const DataPoint& point2 = data_[index2];
        float heading1_x = std::cos(point1.get_yaw());
        float heading1_y = std::sin(point1.get_yaw());
        float heading2_x = std::cos(point2.get_yaw());
        float heading2_y = std::sin(point2.get_yaw());
        float dot_product = heading1_x * heading2_x + heading1_y * heading2_y;
        return std::acos(dot_product);
    }

    std::pair<int, float> closest_point(const DataPoint& point, float angle) const
    {
        float min_distance = std::numeric_limits<float>::max();
        int closest_index = -1;

        for (int i = 0; i < data_.size(); ++i) {
            const DataPoint& other_point = data_[i];
            float dx = other_point.get_x() - point.get_x();
            float dy = other_point.get_y() - point.get_y();
            float dz = other_point.get_z() - point.get_z();
            float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            float heading_x = std::cos(other_point.get_yaw());
            float heading_y = std::sin(other_point.get_yaw());
            float dot_product = heading_x * std::cos(angle) + heading_y * std::sin(angle);
            float angle_diff = std::acos(dot_product);

            if (distance < min_distance && angle_diff < M_PI / 4.0) {
                min_distance = distance;
                closest_index = i;
            }
        }

        return std::make_pair(closest_index, min_distance);
    }

private:
    std::vector<DataPoint> data_;
};
*/


struct GpsCoordinate
{
    double latitude;
    double longitude;
    double altitude;
};
struct EcefCoordinate
{
    double x;
    double y;
    double z;
};
struct XYZ
{
    float x;
    float y;
    float z;
};
struct ROT
{
    float roll;
    float pitch;
    float yaw;
    
};

class DatasetHelpers
{
public:
    // calculates from quaternion to euler angle
    static ROT quaternionToEuler(msr::airlib::Quaternionr& q);
    static void save_DCT(std::bitset<64> DCT, std::ofstream DCT_file);
    static void save_data_to_csv(std::string data_folder, std::string image_name, std::bitset<64> DCT, std::vector<double> SH, XYZ coordinate, ROT rotation);
    static void save_SH(std::vector<double> SH, std::ofstream SH_file);
    static void write_csv(std::ofstream data_file, std::string data_folder);
    static std::vector<double> calculate_spherial_harmonics_coefficents(cv::Mat img_panorama, GPU_SH::SHTransform* sht);
    static std::bitset<64> calculate_dct_hash(cv::Mat& img_pano, GpuDct* gdct);
    static std::bitset<64> hex_to_bitset(const std::string& hex);
    static std::string bitset_to_hex(const std::bitset<64>& bs);
    static std::string saveImage(const std::string& data_folder, unsigned long long time_stamp, const cv::Mat& img, std::string prefix);
    // calculates the reference position from the gps data - the first gps position
    static GpsCoordinate calculate_reference(const msr::airlib::GpsBase::Output& gps_data);

    // get x, y, z coordinates and euler angles from gps and imu data
    static void ll2xyz(msr::airlib::GpsBase::Output gps_data, msr::airlib::ImuBase::Output imu_data, GpsCoordinate& reference_position, XYZ& coordinate, ROT& rotation);

    // converts gps coordinates to ecef coordinates
    static EcefCoordinate gpsToEcef(const GpsCoordinate& gps);
    static XYZ ecefToEnu(const EcefCoordinate& ecef, const EcefCoordinate& ref_ecef, const GpsCoordinate& ref_gps);
    static XYZ gpsToEnu(const GpsCoordinate& gps, const GpsCoordinate& ref_gps);

    static std::string createUniqueFolder();
    static std::string createUniqueFolder(const std::string& rootPath);
    static double* read_sh_data(std::string path, int& num_samples, int num_coeffs);

    static void write_csv(std::string path, std::vector<std::string> header, std::vector<std::vector<float>> data);


    
};
