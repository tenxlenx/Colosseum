#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QCheckBox>
#include <QApplication>
#include <QComboBox>
#include <iostream>
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include <QCommandLinkButton>
#include <QLabel>
#include <QLineEdit>
#include <QListWidgetItem>
#include <QTabWidget>
#include <QTreeWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QJsonParseError>
#include <QVariant>
#include <QVariantMap>
#include <QVariantList>
#include <QVariant>
#include <QMap>
#include "simulation.h"
#include "simulation_tab_widget.h"
#include <nlohmann/json.hpp>
#include "robot_map_widget.h"

void SaveJson(const nlohmann::json& json_value, const std::string& filename)
{
    std::ofstream stream(filename);
    if (!stream.good()) {
        throw std::runtime_error("Failed to open file for writing");
    }
    stream << json_value.dump(4);
}

void addParameterItem(QTreeWidgetItem* parentItem, const QString& key, const QString& value)
{
    QTreeWidgetItem* parameterItem = new QTreeWidgetItem(parentItem);
    parameterItem->setText(0, key);
   
    QTreeWidgetItem* valueItem = new QTreeWidgetItem(parameterItem);
    valueItem->setText(0, value);
    valueItem->setFlags(valueItem->flags() | Qt::ItemIsEditable);
}


// this is getting all the data required for the next view and pass it to the simulation tab
void MainWindow::onSelectMapClicked()
{
    WindowData data;
    data.serverIp = ServerIpEdit->displayText();
    data.serverPort = ServerPortEdit->displayText();
    data.clockSpeed = ClockSpeedEdit->displayText();
    data.simulationBins = SimulationBinsEdit->displayText();
    data.timeOfDay = TimeOfDay->time().toString();
    data.isGpsChecked = GpsCBox->isChecked();
    data.isImuChecked = ImuCBox->isChecked();
    data.isBarometerChecked = BarometerCBox->isChecked();
    data.isMagnetometerChecked = MagnetometerCBox->isChecked();
    data.isLidarChecked = LidarCBox->isChecked();
    data.vehicleType = VehicleType(VehicleSelectorComboBox->currentIndex());
    getCameraData(CameraParametersWidget, data.cameraList);

    qDebug() << "Server IP: " << data.serverIp;
    qDebug() << "Server Port: " << data.serverPort;
    qDebug() << "Clock Speed: " << data.clockSpeed;
    qDebug() << "Simulation Bins: " << data.simulationBins.append("/" + CurrentSelectedMapLabel->text() + "/Windows/");
    qDebug() << "Time of Day: " << data.timeOfDay;
    qDebug() << "GPS: " << (data.isGpsChecked ? "True" : "False");
    qDebug() << "IMU: " << (data.isImuChecked ? "True" : "False");
    qDebug() << "Barometer: " << (data.isBarometerChecked ? "True" : "False");
    qDebug() << "Magnetometer: " << (data.isMagnetometerChecked ? "True" : "False");
    qDebug() << "Lidar: " << (data.isLidarChecked ? "True" : "False");
    qDebug() << "Vehicle Type: " << data.vehicleType;


    std::vector<VehicleCamera> camera_list;
    getCameraData(CameraParametersWidget, camera_list);
    simqt.setCameras(camera_list);
    data.cameraList = camera_list; 

    for (const VehicleCamera& camera : data.cameraList) {
        qDebug() << "Camera name:" << camera.cameraConfig.name;
        qDebug() << "Type: " << camera.cameraType;
        qDebug() << "Width: " << camera.cameraConfig.width;
        qDebug() << "Height: " << camera.cameraConfig.height;
        qDebug() << "X: " << camera.cameraConfig.x;
        qDebug() << "Y: " << camera.cameraConfig.y;
        qDebug() << "Z: " << camera.cameraConfig.z;
        qDebug() << "Roll: " << camera.cameraConfig.roll;
        qDebug() << "Pitch: " << camera.cameraConfig.pitch;
        qDebug() << "Yaw: " << camera.cameraConfig.yaw;
        qDebug() << "Enabled: " << (camera.isEnabled ? "True" : "False");
    }

    
    using json = nlohmann::json;
    json airsim_config;

    // Top-level settings
    airsim_config["SeeDocsAt"] = "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md";
    airsim_config["SettingsVersion"] = 1.2;
    airsim_config["ViewMode"] = "SpringArmChase";
    airsim_config["ClockSpeed"] = std::stof(ClockSpeedEdit->displayText().toStdString());
    airsim_config["SpeedUnitFactor"] = 1.0;
    airsim_config["SpeedUnitLabel"] = "m/s";
    airsim_config["OriginGeopoint"] = {
       {"Latitude", 50.827778},
       {"Longitude", -0.1527785},
       {"Altitude", 60}
    };
    airsim_config["TimeOfDay"] = {
       {"Enabled", false},
       {"StartDateTime", "2022-03-23 22:20:00"},
       {"CelestialClockSpeed", 1},
       {"StartDateTimeDst", false},
       {"UpdateIntervalSecs", 60}
    };
   


    // Vehicle settings
    json vehicle;

    
    vehicle["DefaultVehicleState"] = "Armed";
    vehicle["EnableCollisionPassthrogh"] = false;
    vehicle["EnableCollisions"] = true;
    vehicle["AllowAPIAlways"] = true;

    // Sensor settings
    json sensors;
    sensors["Gps"]["SensorType"] = 3;
    sensors["Gps"]["Enabled"] = GpsCBox->isChecked();
    sensors["Barometer"]["SensorType"] = 1;
    sensors["Barometer"]["Enabled"] = BarometerCBox->isChecked();
    sensors["Magnetometer"]["SensorType"] = 4;
    sensors["Magnetometer"]["Enabled"] = MagnetometerCBox->isChecked();
    sensors["Imu"]["SensorType"] = 2;
    sensors["Imu"]["Enabled"] = ImuCBox->isChecked();
    vehicle["Sensors"] = sensors;

    // Camera settings
    // Note: You'll need to translate `getCameraData(CameraParametersWidget, data.cameraList);` to get actual camera settings
    for (int i = 0; i < data.cameraList.size(); i++) {
        auto cam = data.cameraList[i];
        json cameraj_json;
        json capture_settings;
        capture_settings["ImageType"] = 10;
        capture_settings["Width"] = cam.cameraConfig.width;
        capture_settings["Height"] = cam.cameraConfig.height;
        capture_settings["AutoExposureSpeed"] = 100;
        capture_settings["AutoExposureBias"] = 0;
        capture_settings["AutoExposureMaxBrightness"] = 0.64;
        capture_settings["AutoExposureMinBrightness"] = 0.03;
        capture_settings["MotionBlurAmount"] = 0;
        capture_settings["TargetGamma"] = 2.66;
        cameraj_json["CaptureSettings"][0] = capture_settings;
        cameraj_json["X"] = cam.cameraConfig.x;
        cameraj_json["Y"] = cam.cameraConfig.y;
        cameraj_json["Z"] = cam.cameraConfig.z;
        cameraj_json["Pitch"] = cam.cameraConfig.pitch;
        cameraj_json["Roll"] = cam.cameraConfig.roll;
        cameraj_json["Yaw"] = cam.cameraConfig.yaw;
        auto camera_name = cam.cameraConfig.name;
        vehicle["Cameras"][camera_name] = cameraj_json;
    }

    // Add vehicle to vehicles list
    if (data.vehicleType == VehicleType::CAR) {
        vehicle["VehicleType"] = "PhysXCar";
        airsim_config["Vehicles"]["PhysXCar"] = vehicle;
        airsim_config["SimMode"] = "Car";
    }
    else if (data.vehicleType == VehicleType::DRONE) {
        vehicle["VehicleType"] = "SimpleFlight";
        airsim_config["Vehicles"]["SimpleFlight"] = vehicle;
        airsim_config["SimMode"] = "Multirotor";
    }
  

    QString exepath = data.simulationBins;
    simqt.setServerPath(exepath.toStdString());
    
    std::cout << airsim_config.dump(4);


    currentConfig = airsim_config;
    SimulationTab = new SimulationTabWidget(0, ConnectionStatusLabel, CameraViewLabel, gameImageViewLabel); // passing ui elements
    SimulationTab->setSimulationData(data);
    SimulationTab->setMapWidget(MapViewWidget);
    SimulationTab->setMapName(CurrentSelectedMapLabel->text().toStdString());
    connect(StartServerButton, &QPushButton::clicked, SimulationTab, &SimulationTabWidget::onStartServerClicked);
    connect(StopServerButton, &QPushButton::clicked, SimulationTab, &SimulationTabWidget::onStopServerClicked);
    
}




void MainWindow::onSaveConfigClicked()
{
    auto path = simqt.getServerPath();
    auto jsonpath = path.append("/settings.json");
    SaveJson(currentConfig, jsonpath);
    
}

void MainWindow::onRecordRouteClicked()
{
    // Handle MenuRecordRoute action
}

void MainWindow::onLoadRouteClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open File"),
                                                    "",
                                                    tr("CSV Files (*.csv)"));
    if (!fileName.isEmpty()) {
        qDebug() << "Selected .csv file: " << fileName;
        currentLoadedRoutePath = fileName;
        emit fileSelected(fileName);
    }
}

void MainWindow::onMenuSaveConfigClicked()
{
    // Handle MenuSaveConfig action
}

void MainWindow::onMenuLoadConfigClicked()
{
    // Handle MenuLoadConfig action
}

void MainWindow::onMenuNewSimulationClicked()
{
    // Handle MenuNewSimulation action
}

void MainWindow::onNewCameraAddClicked()
{
	// Handle NewCameraAddButton click event
    addCameraItem(CameraParametersWidget, "camera");
}

void MainWindow::addCameraItem(QTreeWidget* treeWidget, const QString& cameraName)
{
    QTreeWidgetItem* cameraItem = new QTreeWidgetItem(treeWidget);
    cameraItem->setText(0, cameraName);
    cameraItem->setFlags(cameraItem->flags() | Qt::ItemIsEditable);

    // Create child items for each camera parameter with default values
    addParameterItem(cameraItem, "Width", "0");
    addParameterItem(cameraItem, "Height", "0");
    addParameterItem(cameraItem, "X", "0.0");
    addParameterItem(cameraItem, "Y", "0.0");
    addParameterItem(cameraItem, "Z", "0.0");
    addParameterItem(cameraItem, "Roll", "0.0");
    addParameterItem(cameraItem, "Pitch", "0.0");
    addParameterItem(cameraItem, "Yaw", "0.0");
    addParameterItem(cameraItem, "Enabled", "false");
    addParameterItem(cameraItem, "Type", "Scene");
}

void MainWindow::getCameraData(QTreeWidget* treeWidget, std::vector<VehicleCamera>& cameraList)
{
    // Clear the camera list to start with a clean slate
    cameraList.clear();

    // Iterate through top-level items (cameras)
    int topLevelItemCount = treeWidget->topLevelItemCount();
    for (int i = 0; i < topLevelItemCount; ++i) {
        QTreeWidgetItem* cameraItem = treeWidget->topLevelItem(i);
        QString cameraName = cameraItem->text(0);

        // Create a new VehicleCamera object
        VehicleCamera camera;
        camera.cameraConfig.name = cameraName.toStdString();
  
        // Iterate through child items (fields) of the camera
        int fieldCount = cameraItem->childCount();
        for (int j = 0; j < fieldCount; ++j) {
            QTreeWidgetItem* fieldItem = cameraItem->child(j);
            QString fieldKey = fieldItem->text(0);

            // Retrieve the value (child of child item)
            if (fieldItem->childCount() > 0) {
                QTreeWidgetItem* valueItem = fieldItem->child(0);
                QString value = valueItem->text(0);

                // Assign the value to the corresponding CameraConfig field
                if (fieldKey == "Width")
                    camera.cameraConfig.width = value.toInt();
                else if (fieldKey == "Height")
                    camera.cameraConfig.height = value.toInt();
                else if (fieldKey == "X")
                    camera.cameraConfig.x = value.toFloat();
                else if (fieldKey == "Y")
                    camera.cameraConfig.y = value.toFloat();
                else if (fieldKey == "Z")
                    camera.cameraConfig.z = value.toFloat();
                else if (fieldKey == "Roll")
                    camera.cameraConfig.roll = value.toFloat();
                else if (fieldKey == "Pitch")
                    camera.cameraConfig.pitch = value.toFloat();
                else if (fieldKey == "Yaw")
                    camera.cameraConfig.yaw = value.toFloat();
                else if (fieldKey == "Enabled")
                    camera.isEnabled = (value == "true");
				else if (fieldKey == "Type")
					camera.cameraType = CameraType(value.toInt());
            }
        }

        // Add the camera to the camera list
        cameraList.push_back(camera);
    }
}


void populateListWidgetWithDirectories(const QString& path, QListWidget* listWidget)
{
    // Clear the list widget to avoid duplicates if needed
    listWidget->clear();

    QDir directory(path);
    if (!directory.exists()) {
        // Handle the case when the given path does not exist
        return;
    }

    directory.setFilter(QDir::Dirs | QDir::Files | QDir::NoDotAndDotDot); // Include directories and files (excluding '.' and '..')
    const QFileInfoList& entries = directory.entryInfoList();

    // Populate the list widget with the directories and file names
    for (const QFileInfo& entry : entries) {
        QString name = entry.fileName();
        QListWidgetItem* item = new QListWidgetItem(name);
        listWidget->addItem(item);
    }
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
   
    //Tab configuration and views
    AnlysisTab = ui->AnalysisTab;
   // CameraViewWidget = ui->CameraViewWidget;
    MapViewWidget = ui->MapViewWidget;
    
    LoadMapWidget = ui->LoadMapWidget;
    TabWidget = ui->TabWidget;


    //Vehicle configuration
    LidarCBox = ui->LidarCBox;
    BarometerCBox = ui->BarometerCBox;
    GpsCBox = ui->GpsCBox;
    ImuCBox = ui->ImuCBox;
    MagnetometerCBox = ui->MagnetometerCBox;
    //server configuration
    ServerIpEdit = ui->ServerIpEdit;
    ServerPortEdit = ui->ServerPortEdit;
    ClockSpeedEdit = ui->ClockSpeedEdit;
    SimulationBinsEdit = ui->SimulationBinsEdit;
    RouteDbPathEdit = ui->RouteDbPathEdit;
    ConfigPathEdit = ui->ConfigPathEdit;
    VehicleSelectorComboBox = ui->VehicleSelectorComboBox;
    CameraParametersWidget = ui->CameraParametersWidget;
    TimeOfDay = ui->TimeOfDay;

    // buttons 
    StartServerButton = ui->StartServerButton;
    SelectMapButton = ui->SelectMapButton;
    SaveConfigButton = ui->SaveConfigButton;
    StopServerButton = ui->StopServerButton;
    //Menu bar
    MenuRecordRoute = ui->MenuRecordRoute;
    MenuLoadRoute = ui->MenuLoadRoute;
    MenuSaveConfig = ui->MenuSaveConfig;
    MenuLoadConfig = ui->MenuLoadConfig;
    MenuNewSimulation = ui->MenuNewSimulation;
    MenuNewCameraAdd = ui->MenuNewCameraAdd;

    MenuFollowRoute = ui->MenuFollowRoute;
    // Labels 
    ConnectionStatusLabel = ui->ConnectionStatusLabel;
    CurrentSelectedMapLabel = ui->CurrentSelectedMapLabel;
    gameImageViewLabel = ui->gameImageViewLabel;

    // image views
    CameraViewLabel = ui->CameraViewLabel;

  
    


    LoadMapWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(SimulationBinsEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
        populateListWidgetWithDirectories(text, LoadMapWidget);
    });


    ServerIpEdit->setText("127.0.0.1");
    ServerPortEdit->setText("41451");
    ClockSpeedEdit->setText("1.0");
    SimulationBinsEdit->setText("D:/SIMULATION_BINARIES");
    TimeOfDay->setTime(QTime(16, 20));

    // set camera parameters
    // Set default values for camera member variables
    VehicleCamera defaultCamera {};
    defaultCamera.cameraConfig.width = 1024;
    defaultCamera.cameraConfig.height = 1024;
    defaultCamera.cameraConfig.x = 0.0f;
    defaultCamera.cameraConfig.y = 0.0f;
    defaultCamera.cameraConfig.z = -1.25f;
    defaultCamera.cameraConfig.roll = 0.0f;
    defaultCamera.cameraConfig.pitch = 0.0f;
    defaultCamera.cameraConfig.yaw = 90.0f;
    defaultCamera.isEnabled = true;
    defaultCamera.cameraType = CameraType::Panoramic;
    simqt.addCamera(defaultCamera);

    for (const VehicleCamera& camera : simqt.getCameras()) {
        QTreeWidgetItem* cameraItem = new QTreeWidgetItem(CameraParametersWidget);

        // Set the camera name as the top-level item's text
        QString cameraName = "panorama";
        cameraItem->setText(0, cameraName);

        // Create child items for each camera parameter
        addParameterItem(cameraItem, "Width", QString::number(camera.cameraConfig.width));
        addParameterItem(cameraItem, "Height", QString::number(camera.cameraConfig.height));
        addParameterItem(cameraItem, "X", QString::number(camera.cameraConfig.x));
        addParameterItem(cameraItem, "Y", QString::number(camera.cameraConfig.y));
        addParameterItem(cameraItem, "Z", QString::number(camera.cameraConfig.z));
        addParameterItem(cameraItem, "Roll", QString::number(camera.cameraConfig.roll));
        addParameterItem(cameraItem, "Pitch", QString::number(camera.cameraConfig.pitch));
        addParameterItem(cameraItem, "Yaw", QString::number(camera.cameraConfig.yaw));
        addParameterItem(cameraItem, "Enabled", camera.isEnabled ? "True" : "False");
        addParameterItem(cameraItem, "Type", QString::number(camera.cameraType));
    }


    // Connect the StartServerButton click event <---- SimulationTabWidget / tab B
    

    // Connect the SelectMapButton click event
    // connect(ui->SelectMapButton, &QCommandLinkButton::clicked, this, &MainWindow::onSelectMapClicked);
    connect(ui->MenuFollowRoute, &QAction::triggered, SimulationTab, &SimulationTabWidget::onStartAgent);

    connect(ui->SaveConfigButton, &QCommandLinkButton::clicked, this, &MainWindow::onSaveConfigClicked);
    connect(ui->MenuRecordRoute, &QAction::triggered, this, &MainWindow::onRecordRouteClicked);
    connect(ui->MenuLoadRoute, &QAction::triggered, this, &MainWindow::onLoadRouteClicked);
    connect(ui->MenuSaveConfig, &QAction::triggered, this, &MainWindow::onMenuSaveConfigClicked);
    connect(ui->MenuLoadConfig, &QAction::triggered, this, &MainWindow::onMenuLoadConfigClicked);
    connect(ui->MenuNewSimulation, &QAction::triggered, this, &MainWindow::onMenuNewSimulationClicked);
    connect(ui->MenuNewCameraAdd, &QAction::triggered, this, &MainWindow::onNewCameraAddClicked);
    connect(TabWidget, &QTabWidget::tabBarClicked, this, [=](int index) {
        qDebug() << "Tab clicked" << index;
    });

    // connect the load route to the map widget
    connect(this, &MainWindow::fileSelected, MapViewWidget, &RobotMapWidget::loadPointsFromCSV);

    // Connect the itemClicked signal of the LoadMapListWidget
    connect(SelectMapButton, &QPushButton::clicked, this, &MainWindow::onSelectMapClicked);
    connect(LoadMapWidget, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
    // Initialize and populate the data structure with the required values
        QString selectedItemText = item->text();
        qDebug() << selectedItemText;
        CurrentSelectedMapLabel->setText(selectedItemText);
        CurrentSelectedMapLabel->show();
    });

   
   
}

MainWindow::~MainWindow()
{
    delete ui;
}

