#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "common/Common.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "rpc/client.h"
#include "common/common_utils/FileSystem.hpp"

#include "ImageThread.h"
#include "gpu_dct.cuh"
#include "sh_transform.cuh"
#include <QtGui>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibAdaptors.hpp>

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "DatasetHelpers.h"
#include "rpc/detail/response.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"



ImageThread::ImageThread(QObject* parent, GpuDct* gdct, GPU_SH::SHTransform* sh, bool isCarMode, QLabel *cameraViewLabel, const std::string &simulation_folder_path)
    : run_flag(true)
    , ref_gps()
    , _label(cameraViewLabel)
    , _simulation_folder_path(simulation_folder_path)
    , _client_multirotor(nullptr)
    , _gdct(gdct)
    , _sht(sh)
{
    ref_gps_initialized = false;
    _data_folder = DatasetHelpers::createUniqueFolder(simulation_folder_path);
    _data_file.open(_data_folder + "/sensor_data.csv", std::ios::app);
    _DCT_file.open(_data_folder + "/dct_data.csv", std::ios::app);
    _SH_file.open(_data_folder + "/sh_data.csv", std::ios::app);


    _data_file << "timestamp,x,y,z,yaw,pitch,roll,vel_x,vel_y,vel_z,ang_vel_x, ang_vel_y, ang_vel_z, image_name" << std::endl;
    connect(this, &ImageThread::imageReady, this, &ImageThread::display_image);
    
}

void ImageThread::display_image(const QPixmap& pixmap)
{

    _label->setPixmap(pixmap.scaled(_label->width(), _label->height(), Qt::KeepAspectRatio));
    
}


void ImageThread::save_SH(std::vector<double> SH)
{
    
    _SH_file.open(_data_folder + "/sh_data.csv", std::ios::app);
    for (int s = 0; s < SH.size(); s++) {
        _SH_file << SH[s] << ",";
    }
    _SH_file << std::endl;
}

void ImageThread::save_DCT(std::bitset<64> DCT)
{
    
    _DCT_file.open(_data_folder + "/dct_data.csv", std::ios::app);
    _DCT_file << DatasetHelpers::bitset_to_hex(DCT) << std::endl;
    _DCT_file.close();
}

void ImageThread::write_csv_header()
{
    _data_file.open(_data_folder + "/sensor_data.csv", std::ios::app);
    _data_file << "timestamp,vehicle_type,x,y,z,yaw,pitch,roll,speed,steering,image_name" << std::endl;
    _data_file.close();
}

bool ImageThread::process_sensors(std::vector<ImageRequest> &request,
                                  std::vector<ImageResponse> &response,
                                  msr::airlib::GpsBase::Output &gps_data,
                                  msr::airlib::ImuBase::Output &imu_data) const
{

    if (_isCarMode) {
        response = _client->simGetImages(request);
        gps_data = _client->getGpsData();
        imu_data = _client->getImuData();
        return true;
    }
    else if(!_isCarMode) {
        response = _client_multirotor->simGetImages(request);
        gps_data = _client_multirotor->getGpsData();
        imu_data = _client_multirotor->getImuData();
        return true;
    }
    // if we get here, something went wrong
    return false;
}

std::string ImageThread::getDataFolder()
{
    return _data_folder;
}


void ImageThread::run()
{
    int count = 0;

     if (_isCarMode) {
        _client = new msr::airlib::CarRpcLibClient();
        _client->confirmConnection();
    }
    else {
        _client_multirotor = new msr::airlib::MultirotorRpcLibClient();
        _client_multirotor->confirmConnection();
    }

    while (run_flag) {
        try {
            std::vector<ImageRequest> img_request = { ImageRequest("panorama", ImageType::CubeScene, false, true) };
    
            std::vector<ImageResponse> response;
            msr::airlib::GpsBase::Output gps_data;
            msr::airlib::ImuBase::Output imu_data;
            std::vector<cv::Mat> images;
            msr::airlib::Vector3r lin_velocity;
            msr::airlib::Vector3r ang_velocity;
            process_sensors(img_request, response, gps_data, imu_data);
            DatasetHelpers::calculate_reference(gps_data);
            msr::airlib::CarApiBase::CarControls car_controls;
            msr::airlib::CarApiBase::CarState car_state;
            msr::airlib_rpclib::MultirotorRpcLibAdaptors::MultirotorState multi_rotor_state;

            /// decode the image from the client 
            cv::Mat img_pano = cv::imdecode(response[0].image_data_uint8, 1);
            const auto time_stamp = response[0].time_stamp;

            /// save the image to the disk
            auto image_name = DatasetHelpers::saveImage(_data_folder, time_stamp, img_pano, "panorama");

            /// calculate the hash and the spherical harmonics
            std::bitset<64> hash_set = DatasetHelpers::calculate_dct_hash(img_pano, _gdct);
            std::vector<double> SH = DatasetHelpers::calculate_spherial_harmonics_coefficents(img_pano, _sht);

            // SAVE SH to CSV
            for (int s = 0; s < SH.size(); s++) {
                _SH_file << SH[s] << ",";
            }
            _SH_file << std::endl;

            // save dct to csv
            _DCT_file << DatasetHelpers::bitset_to_hex(hash_set) << std::endl;

            msr::airlib::Pose position_record;
            /// get the velocity and the orientation of the vehicle
            if (_isCarMode) {
                car_controls = _client->getCarControls();
                car_state = _client->getCarState();
                lin_velocity = car_state.kinematics_estimated.twist.linear;
                ang_velocity = car_state.kinematics_estimated.twist.angular;
                auto position = car_state.getPosition();
                auto orientation = car_state.getOrientation();
                const auto& euler = DatasetHelpers::quaternionToEuler(orientation);
                _data_file << response[0].time_stamp << ","
                           << position.x() << ","
                           << position.y() << ","
                           << position.z() << ","
                           << euler.yaw << ","
                           << euler.pitch << ","
                           << euler.roll << ","
                           << lin_velocity.x() << ","
                           << lin_velocity.y() << ","
                           << lin_velocity.z() << ","
                           << ang_velocity.x() << ","
                           << ang_velocity.y() << ","
                           << ang_velocity.z() << ","
                           << image_name << std::endl;
                position_record = _client->simGetVehiclePose();
                
            }
            else {
                multi_rotor_state = _client_multirotor->getMultirotorState();
                lin_velocity = multi_rotor_state.kinematics_estimated.linear_velocity.to();
                ang_velocity = multi_rotor_state.kinematics_estimated.angular_velocity.to();

                auto position = _client_multirotor->getMultirotorState().getPosition();
                auto orientation = _client_multirotor->getMultirotorState().getOrientation();
                const auto& euler = DatasetHelpers::quaternionToEuler(orientation);
                _data_file << response[0].time_stamp << ","
                           << position.x() << ","
                           << position.y() << ","
                           << position.z() << ","
                           << euler.yaw << ","
                           << euler.pitch << ","
                           << euler.roll << ","
                           << lin_velocity.x() << ","
                           << lin_velocity.y() << ","
                           << lin_velocity.z() << ","
                           << ang_velocity.x() << ","
                           << ang_velocity.y() << ","
                           << ang_velocity.z() << ","
                           << image_name << std::endl;
                position_record = _client_multirotor->simGetVehiclePose();
                
            }

            /// showing image in gui
            QImage qimage(img_pano.data, img_pano.cols, img_pano.rows, img_pano.step, QImage::Format_BGR888);
            QPixmap pixmap = QPixmap::fromImage(qimage);
            emit imageReady(pixmap);
            emit positionReady(position_record.position.x(), position_record.position.y());
 
            count++;
        }
        catch (rpc::rpc_error& e) {
            std::string msg = "Exception raised by the API, something went wrong in image thread.\n" + e.get_error().as<std::string>();
            QString qmsg(msg.c_str());
            qDebug() << qmsg;
        }
    }
    this->deleteLater();
}