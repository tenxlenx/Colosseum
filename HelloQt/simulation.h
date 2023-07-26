#pragma once
#ifndef SIMULATION_H
#define SIMULATION_H

struct CameraConfig
{
    std::string name;
    int width;
    int height;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

enum CameraType
{
    Scene = 0,
    DepthPlanar = 1,
    DepthPerspective = 2,
    DepthVis = 3,
    DisparityNormalized = 4,
    Segmentation = 5,
    SurfaceNormals = 6,
    Infrared = 7,
    OpticalFlow = 8,
    OpticalFlowVis = 9,
    Panoramic = 10
};

struct VehicleCamera
{
    CameraConfig cameraConfig;
    bool isEnabled;
    CameraType cameraType;
};

enum VehicleType
{
    CAR = 0,
    DRONE = 1
};

struct VehicleConfig
{
    VehicleType type;
    bool enabled;
    bool hasGPS;
    bool hasLidar;
    bool hasBarometer;
    bool hasImu;
    bool hasMagnetometer;
    std::vector<VehicleCamera> cameras;
};

struct ServerConfig
{
    std::string ip;
    int port;
    int clockSpeed;
};

struct SimulationConfig
{
    std::string mapPath;
    std::string routePath;
    std::string configPath;
    VehicleConfig vehicleConfig;
    ServerConfig serverConfig;
};

struct WindowData
{
    QString selectedItemText;
    QString serverIp;
    QString serverPort;
    QString clockSpeed;
    QString simulationBins;
    QString timeOfDay;
    bool isGpsChecked;
    bool isImuChecked;
    bool isBarometerChecked;
    bool isMagnetometerChecked;
    bool isLidarChecked;
    enum VehicleType vehicleType;
    SimulationConfig simulationConfig;
    std::vector<VehicleCamera> cameraList;
};


class SimulationData
{
public:
    SimulationData() {}
    ~SimulationData() {}

    void setServerIp(std::string ip) { server_ip = ip; }
    void setServerPort(std::string port) { server_port = port; }
    void setServerPath(std::string path) { server_path = path; }
    void setClockSpeed(float speed) { clock_speed = speed; }
    void setGps(bool gps) { vehicleConfig.hasGPS  = gps; }
    void setImu(bool imu) { vehicleConfig.hasImu = imu; }
    void setLidar(bool lidar) { vehicleConfig.hasLidar = lidar; }
    void setMagnetometer(bool magnetometer) { vehicleConfig.hasMagnetometer = magnetometer; }
    void setBarometer(bool barometer) { vehicleConfig.hasBarometer = barometer; }
    void setCameras(std::vector<VehicleCamera> cameras) { this->vehicleConfig.cameras = cameras; }
    void addCamera(VehicleCamera camera) { vehicleConfig.cameras.push_back(camera); }
    std::vector<VehicleCamera> getCameras() { return vehicleConfig.cameras; }
    std::string getServerIp() { return server_ip; }
    std::string getServerPort() { return server_port; }
    std::string getServerPath() { return server_path; }
    float getClockSpeed() { return clock_speed; }
    bool getGps() { return vehicleConfig.hasGPS; }
    bool getImu() { return vehicleConfig.hasImu; }
    bool getLidar() { return vehicleConfig.hasLidar; }
    bool getMagnetometer() { return vehicleConfig.hasMagnetometer; }
    bool getBarometer() { return vehicleConfig.hasBarometer; }



private:
    std::string server_ip;
    std::string server_port;
    std::string server_path;
    float clock_speed;
    VehicleConfig vehicleConfig;

};

class SimulationLogic
{
public:
    void startServer();


private:
};

#endif // SIMULATION_H