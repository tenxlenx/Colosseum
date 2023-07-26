#pragma once
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLabel>
#include <QPixmap>
#include "gpu_dct.cuh"
#include "sh_transform.cuh"
#include <fstream>
#include "ImageThread.h"
#include "robot_map_widget.h"
#include "common/common_utils/Utils.hpp"
#include "common/Common.hpp"

class CameraViewWidget : public QWidget
{
    Q_OBJECT
public:
    CameraViewWidget(QWidget* parent = nullptr, QLabel* cameraViewLabel = 0);
    void startDataVis(std::string mapName);
    ~CameraViewWidget();
    void setMapWidget(RobotMapWidget* mapWidget);

private:

    QLabel* m_cameraViewLabel;
    ImageThread* imageThread;
    GPU_SH::SHTransform* SHT;
    std::shared_ptr<GpuDct> DCT;
    cublasHandle_t _handle;
    std::ofstream data_file;
    std::ofstream SH_file;
    bool isCarVehicle;
    RobotMapWidget* robotMapWidget;

    

};