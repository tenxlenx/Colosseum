#pragma once

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include "common/common_utils/StrictMode.hpp"
#include "rpc/client.h"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "common/common_utils/FileSystem.hpp"
#include "gpu_dct.cuh"
#include "sh_transform.cuh"
#include <QtGui>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include <QThread>
#include <QtCore/QtCore>
#include <QtWidgets/QtWidgets>
#include "DatasetHelpers.h"
#include "common/Common.hpp"


class ImageThread : public QThread
{
    Q_OBJECT
public:
    typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
    typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    bool run_flag;

    explicit ImageThread(QObject* parent, GpuDct* gdct, GPU_SH::SHTransform* sh, bool isCarMode, QLabel* cameraViewLabel, const std::string &simulation_folder_path);
    void save_SH(std::vector<double> SH);
    void save_DCT(std::bitset<64> DCT);
    void set_client_parameters(std::string& data_folder, bool& isCarMode, std::ofstream& data_file, std::ofstream& SH_file, std::ofstream& test_SH);
    void write_csv_header();
    bool process_sensors(std::vector<ImageRequest>& request, std::vector<ImageResponse>& response, msr::airlib::GpsBase::Output& gps_data, msr::airlib::ImuBase::Output& imu_data) const;
    std::string getDataFolder();

    signals:
    void imageReady(const QPixmap& pixmap);
    void positionReady(float x, float y);

    

public slots:
    void run() override;
    void display_image(const QPixmap& pixmap);

private:

    std::string _simulation_folder_path;
    bool ref_gps_initialized;
    GpsCoordinate ref_gps;
    QLabel* _label;
    msr::airlib::CarRpcLibClient* _client;
    msr::airlib::MultirotorRpcLibClient* _client_multirotor;
    GpuDct* _gdct;
    GPU_SH::SHTransform* _sht;
    std::string _data_folder;
    bool _isCarMode = false;
    std::ofstream _data_file, _SH_file, _DCT_file;
  
};
