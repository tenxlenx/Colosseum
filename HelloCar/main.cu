// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include "gpu_dct.h"


constexpr double kSemiMajorAxis = 6378137.0;
constexpr double kSemiMinorAxis = 6356752.3142;
constexpr double kEccentricity = 8.1819190842622e-2;

struct ImageWriteTask
{
    std::string file_path;
    cv::Mat image;
};




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

struct CartesianCoordinate
{
    double x;
    double y;
    double z;
};

EcefCoordinate gpsToEcef(const GpsCoordinate& gps)
{
    double lat_rad = gps.latitude * M_PI / 180.0;
    double lon_rad = gps.longitude * M_PI / 180.0;
    double N = kSemiMajorAxis / std::sqrt(1 - kEccentricity * kEccentricity * std::sin(lat_rad) * std::sin(lat_rad));
    double x = (N + gps.altitude) * std::cos(lat_rad) * std::cos(lon_rad);
    double y = (N + gps.altitude) * std::cos(lat_rad) * std::sin(lon_rad);
    double z = ((1 - kEccentricity * kEccentricity) * N + gps.altitude) * std::sin(lat_rad);
    return { x, y, z };
}

CartesianCoordinate ecefToEnu(const EcefCoordinate& ecef, const EcefCoordinate& ref_ecef, const GpsCoordinate& ref_gps)
{
    double lat_rad = ref_gps.latitude * M_PI / 180.0;
    double lon_rad = ref_gps.longitude * M_PI / 180.0;

    double dx = ecef.x - ref_ecef.x;
    double dy = ecef.y - ref_ecef.y;
    double dz = ecef.z - ref_ecef.z;

    double x = -std::sin(lon_rad) * dx + std::cos(lon_rad) * dy;
    double y = -std::sin(lat_rad) * std::cos(lon_rad) * dx - std::sin(lat_rad) * std::sin(lon_rad) * dy + std::cos(lat_rad) * dz;
    double z = std::cos(lat_rad) * std::cos(lon_rad) * dx + std::cos(lat_rad) * std::sin(lon_rad) * dy + std::sin(lat_rad) * dz;

    return { x, y, z };
}

CartesianCoordinate gpsToEnu(const GpsCoordinate& gps, const GpsCoordinate& ref_gps)
{
    EcefCoordinate ecef = gpsToEcef(gps);
    EcefCoordinate ref_ecef = gpsToEcef(ref_gps);
    return ecefToEnu(ecef, ref_ecef, ref_gps);
}

struct EulerAngle
{
    double roll;
    double pitch;
    double yaw;
};

EulerAngle quaternionToEuler(const msr::airlib::Quaternionr& q)
{
    double roll = std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    double pitch = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    return { roll, pitch, yaw };
}

std::string createUniqueFolder()
{
    auto now = std::chrono::system_clock::now();
    auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

    std::stringstream folder_name_ss;
    folder_name_ss << "data_" << seconds_since_epoch;

    std::string folder_name = folder_name_ss.str();
    std::filesystem::create_directories(folder_name);

    return folder_name;
}

void image_writer_thread_func(
    std::queue<ImageWriteTask>& task_queue,
    std::mutex& task_queue_mutex,
    std::condition_variable& task_queue_cv,
    bool& stop_thread)
{
    while (true) {
        std::unique_lock<std::mutex> lock(task_queue_mutex);
        task_queue_cv.wait(lock, [&] { return !task_queue.empty() || stop_thread; });

        if (stop_thread && task_queue.empty()) {
            break;
        }

        ImageWriteTask task = task_queue.front();
        task_queue.pop();
        lock.unlock();

        cv::imwrite(task.file_path, task.image);
    }
}


int main()
{
    using namespace msr::airlib;
    bool isCarMode = false;

    std::cout << "Make sure settings.json has \"SimMode\"=\"Car\" at root. Press Enter to continue." << std::endl;
    std::cin.get();

    msr::airlib::CarRpcLibClient client("127.0.0.1");

    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    auto vehicles = client.listVehicles();
    for (auto v : vehicles) {
        std::cout << "VEHICLE: " << v << std::endl;
    }

    if (vehicles[0] == "drone_flight") {
        isCarMode = false;
    }
    else {
        isCarMode = true;
    }

    std::string data_folder = createUniqueFolder();
    std::ofstream data_file;
    data_file.open(data_folder + "/sensor_data.csv");
    if (isCarMode) {
        data_file << "timestamp,vehicle_type,x,y,z,yaw,pitch,roll,speed,steering,image_name" << std::endl;
    }
    else {
        data_file << "timestamp,vehicle_type,x,y,z,yaw,pitch,roll,speed,steering,image_name" << std::endl;
    }

    GpsCoordinate ref_gps;
    bool ref_gps_initialized = false;

     // Create the image writer thread
    std::queue<ImageWriteTask>image_write_task_queue;
    std::mutex image_write_task_queue_mutex;
    std::condition_variable image_write_task_queue_cv;
    bool stop_image_writer_thread = false;

    std::thread image_writer_thread(
        image_writer_thread_func,
        std::ref(image_write_task_queue),
        std::ref(image_write_task_queue_mutex),
        std::ref(image_write_task_queue_cv),
        std::ref(stop_image_writer_thread));

    try {
        client.confirmConnection();

        while (1) {
            vector<ImageRequest> request = { ImageRequest("panorama", ImageType::CubeScene, false, true) };
            const vector<ImageResponse>& response = client.simGetImages(request);
            cv::Mat h_result = cv::imdecode(response[0].image_data_uint8, 1);

            auto gps_data = client.getGpsData();
            auto imu_data = client.getImuData();
            EulerAngle euler_angles = quaternionToEuler(imu_data.orientation);
            double heading = euler_angles.yaw * 180.0 / M_PI; // Convert radians to degrees
            double pitch = euler_angles.pitch * 180.0 / M_PI;
            double roll = euler_angles.pitch * 180 / M_PI;

            if (!ref_gps_initialized) {
                ref_gps = GpsCoordinate{ gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude };
                ref_gps_initialized = true;
            }

            GpsCoordinate current_gps{ gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude };
            CartesianCoordinate local_xyz = gpsToEnu(current_gps, ref_gps);

            std::string image_name = data_folder + "/panorama_" + std::to_string(response[0].time_stamp) + ".jpg";
            // Add the image write task to the queue
            {
                std::lock_guard<std::mutex> lock(image_write_task_queue_mutex);
                image_write_task_queue.push(ImageWriteTask{ image_name, h_result });
            }
            image_write_task_queue_cv.notify_one();

            if (isCarMode) {
                auto car_controls = client.getCarControls();
                auto car_state = client.getCarState();
                data_file << response[0].time_stamp << ","
                          << "CAR" << ","                                            
                          << local_xyz.x << ","
                          << local_xyz.y << ","
                          << local_xyz.z << ","
                          << heading << ","
                          << pitch << ","
                          << roll << ","
                          << car_state.speed << ","
                          << car_controls.steering << ","
                          << image_name << std::endl;
            }
            else {
                data_file << response[0].time_stamp << ","
                          << local_xyz.x << ","
                          << local_xyz.y << ","
                          << local_xyz.z << ","
                          << heading << ","
                          << pitch << ","
                          << roll << ","
                          << 0 << ","
                          << 0 << ","
                          << image_name << std::endl;
            }
            cv::imshow("pano", h_result);
            cv::waitKey(1);

            
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
        std::cin.get();
    }

    data_file.close();
    return 0;
}
