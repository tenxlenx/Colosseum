#pragma once
#include <gpu_dct.cuh>
#include <sh_transform.cuh>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "common/common_utils/StrictMode.hpp"
#include "rpc/detail/response.h"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "DatasetHelpers.h"

#include <filesystem>
#include <math.h>
#include <fstream>
#include <common/Common.hpp>
#include <vector>
#include "sensors/gps/GpsBase.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void DatasetHelpers::save_SH(std::vector<double> SH, std::ofstream SH_file)
{
    for (int s = 0; s < SH.size(); s++) { SH_file << SH[s] << ","; } SH_file << std::endl;
}

void DatasetHelpers::save_DCT(std::bitset<64> DCT, std::ofstream DCT_file)
{
    DCT_file << bitset_to_hex(DCT) << std::endl;
}

void DatasetHelpers::write_csv(std::ofstream data_file,  std::string data_folder)
{
    data_file.open(data_folder + "/sensor_data.csv");
    //SH_file.open(data_folder + "/sh_data.csv");

    data_file << "timestamp,vehicle_type,x,y,z,yaw,pitch,roll,speed,steering,image_name" << std::endl;
    
}

std::vector<double> DatasetHelpers::calculate_spherial_harmonics_coefficents(cv::Mat img_panorama, GPU_SH::SHTransform* sht)
{
    cv::Mat SH_mat;
    cv::resize(img_panorama, SH_mat, { 128, 64 });
    SH_mat.convertTo(SH_mat, CV_64F, 1.0 / 255);
    std::vector<double> SH = sht->transform(SH_mat);
    return SH;
}

std::bitset<64> DatasetHelpers::calculate_dct_hash(cv::Mat& img_pano, GpuDct *gdct)
{
    cv::Mat gray_res, resized_gray;
    cv::cvtColor(img_pano, gray_res, cv::COLOR_BGR2GRAY);
    cv::resize(gray_res, resized_gray, { 256, 256 }, cv::INTER_CUBIC);
    cv::Mat conv;
    resized_gray.convertTo(conv, CV_32F, 1.0 / 255);
    const unsigned long long int hash = gdct->dct(conv);
    const std::bitset<64> hash_set(hash);
    return hash_set;
}

std::bitset<64> DatasetHelpers::hex_to_bitset(const std::string& hex_string)
{
    std::stringstream ss;
    unsigned long long n;
    ss << std::hex << hex_string;
    ss >> n;
    return { n };
}



std::string DatasetHelpers::bitset_to_hex(const std::bitset<64>& bs)
{
    std::stringstream ss;
    ss << std::hex << bs.to_ullong();
    return ss.str();
}

std::string DatasetHelpers::saveImage(const std::string& data_folder, unsigned long long time_stamp, const cv::Mat& img, std::string prefix)
{
    const std::string image_name = data_folder + "/" + prefix + "_" + std::to_string(time_stamp) + ".png";
    if (!img.empty()) {
        cv::imwrite(image_name, img);
    }
    return image_name;
}


GpsCoordinate DatasetHelpers::calculate_reference(const msr::airlib::GpsBase::Output& gps_data)
{
    return GpsCoordinate{ gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude };
}

void DatasetHelpers::ll2xyz(
            msr::airlib::GpsBase::Output gps_data,
            msr::airlib::ImuBase::Output imu_data,
            GpsCoordinate &reference_position,
            XYZ& coordinate, ROT& rotation)
{
    
    const auto euler = quaternionToEuler(imu_data.orientation);
    rotation.roll = euler.roll;
    rotation.pitch = euler.pitch;
    rotation.yaw = euler.yaw;

    const GpsCoordinate current_gps{ gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude };
    const XYZ xyz = gpsToEnu(current_gps, reference_position);

    coordinate.x = xyz.x;
    coordinate.y = xyz.y;
    coordinate.z = xyz.z;
}

EcefCoordinate DatasetHelpers::gpsToEcef(const GpsCoordinate& gps)
{
    constexpr double kSemiMajorAxis = 6378137.0;
    constexpr double kSemiMinorAxis = 6356752.3142;
    constexpr double kEccentricity = 8.1819190842622e-2;

    const double lat_rad = gps.latitude * M_PI / 180.0;
    const double lon_rad = gps.longitude * M_PI / 180.0;
    double N = kSemiMajorAxis / std::sqrt(1 - kEccentricity * kEccentricity * std::sin(lat_rad) * std::sin(lat_rad));
    const double x = (N + gps.altitude) * std::cos(lat_rad) * std::cos(lon_rad);
    const double y = (N + gps.altitude) * std::cos(lat_rad) * std::sin(lon_rad);
    const double z = ((1 - kEccentricity * kEccentricity) * N + gps.altitude) * std::sin(lat_rad);
    return { x, y, z };
}

XYZ DatasetHelpers::ecefToEnu(const EcefCoordinate& ecef, const EcefCoordinate& ref_ecef, const GpsCoordinate& ref_gps)
{
    const double lat_rad = ref_gps.latitude * M_PI / 180.0;
    const double lon_rad = ref_gps.longitude * M_PI / 180.0;

    const double dx = ecef.x - ref_ecef.x;
    const double dy = ecef.y - ref_ecef.y;
    const double dz = ecef.z - ref_ecef.z;

    const double x = -std::sin(lon_rad) * dx + std::cos(lon_rad) * dy;
    const double y = -std::sin(lat_rad) * std::cos(lon_rad) * dx - std::sin(lat_rad) * std::sin(lon_rad) * dy + std::cos(lat_rad) * dz;
    const double z = std::cos(lat_rad) * std::cos(lon_rad) * dx + std::cos(lat_rad) * std::sin(lon_rad) * dy + std::sin(lat_rad) * dz;

    return { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
}

XYZ DatasetHelpers::gpsToEnu(const GpsCoordinate& gps, const GpsCoordinate& ref_gps)
{
    EcefCoordinate ecef = DatasetHelpers::gpsToEcef(gps);
    EcefCoordinate ref_ecef = gpsToEcef(ref_gps);
    return ecefToEnu(ecef, ref_ecef, ref_gps);
}


ROT DatasetHelpers::quaternionToEuler(msr::airlib::Quaternionr& q)
{
    double roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    double pitch = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    yaw = yaw * 180.0 / M_PI; // Convert radians to degrees
    pitch = pitch * 180.0 / M_PI;
    roll = roll * 180 / M_PI;
    return { static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw) };
}




std::string DatasetHelpers::createUniqueFolder()
{
    auto now = std::chrono::system_clock::now();
    auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

    std::stringstream folder_name_ss;
    folder_name_ss << "data_" << seconds_since_epoch;

    std::string folder_name = folder_name_ss.str();
    std::filesystem::create_directories(folder_name);

    return folder_name;
}



std::string DatasetHelpers::createUniqueFolder(const std::string& rootPath)
{
    std::stringstream folder_path_ss;
    folder_path_ss << rootPath << "/" << createUniqueFolder();

    std::string folder_path = folder_path_ss.str();
    std::filesystem::create_directories(folder_path);

    return folder_path;
}



//---------------------------------------------------------------
double* DatasetHelpers::read_sh_data(std::string path, int& num_samples, int num_coeffs)
{
    std::ifstream infile(path);
    std::vector<double> sh_coeffs;
    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            sh_coeffs.push_back(std::stod(value));
        }
    }
    num_samples = sh_coeffs.size() / num_coeffs;
    auto* sh_coeffs_array = new double[sh_coeffs.size()];
    std::copy(sh_coeffs.begin(), sh_coeffs.end(), sh_coeffs_array);
    return sh_coeffs_array;
}


