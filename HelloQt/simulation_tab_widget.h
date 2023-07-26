#pragma once

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include <QWidget>
#include <QLabel>
#include <QTabWidget>
#include "game_view_widget.h"
#include "simulation.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include "camera_view_widget.h"
#include "robot_map_widget.h"
#include "robot_control.h"

using namespace msr::airlib;
typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;

class SimulationTabWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SimulationTabWidget(QWidget* parent = nullptr, QLabel* status = nullptr, QLabel* cameraViewLabel = nullptr, QLabel* gameImageViewLabel = nullptr);
    ~SimulationTabWidget();
    void setSimulationData(WindowData& simData);
    void setGameViewWidget(GameViewWidget* view);
    void connectToServer(const std::string ip = "127.0.0.1", bool isCarVehicle = false);
    void setMapWidget(RobotMapWidget* mapWidget);
    void setMapName(std::string map_name);

public slots:
    void onStartServerClicked();
    void onStopServerClicked();
    void onStartAgent();
    

    // Add your custom functionality and properties here
private:
    WindowData simData;
    GameViewWidget* gameView; // holds the unreal engine window
    QLabel* connectionStatus;
    std::shared_ptr<msr::airlib::RpcLibClientBase> client;
    const std::string ip_address;
    cublasHandle_t _handle;
    bool isCarVehicle;
    QWidget* interfaceWidget;
    CameraViewWidget *cameraViewWidget;
    QProcess* m_process; // the running unreal engine instance
    std::string map_name;
    RobotMapWidget *robotMapWidget;

    std::shared_ptr<RobotControl> robotControl;
    std::shared_ptr<msr::airlib::VehicleApiBase> _vehicle;
   

};

#include "simulation_tab_widget.moc"




