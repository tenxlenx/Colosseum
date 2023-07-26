
#include <camera_view_widget.h>
#include <ImageThread.h>
#include <QVBoxLayout>
#include "robot_map_widget.h"


CameraViewWidget::CameraViewWidget(QWidget* parent, QLabel* cameraViewLabel)
    : QWidget(parent), m_cameraViewLabel(cameraViewLabel)
{

    
}

void CameraViewWidget::startDataVis(std::string mapName)
{
    std::cout << "Connected " << std::endl;
    QDir dir(QString::fromStdString(mapName));
    cublasCreate(&_handle);
    DCT = std::make_shared<GpuDct>(256);
    DCT->setHandle(_handle);
    SHT = new GPU_SH::SHTransform(128, 64, 8, _handle);
    data_file.open("data.txt");
    SH_file.open("SH.txt");
    imageThread = new ImageThread(this, DCT.get(), SHT, isCarVehicle, m_cameraViewLabel, dir.absolutePath().toStdString());
    imageThread->start();

    connect(imageThread, &ImageThread::positionReady, robotMapWidget, &RobotMapWidget::addPoint);
}

void CameraViewWidget::setMapWidget(RobotMapWidget* mapWidget)
{
    robotMapWidget = mapWidget;
}


CameraViewWidget::~CameraViewWidget()
{
    data_file.close();
    SH_file.close();
    cublasDestroy(_handle);
    delete SHT;
}
