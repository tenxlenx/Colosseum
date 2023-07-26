#include "simulation_tab_widget.h"

#include <memory>
#include <QDebug>
#include "simulation.h"
#include <QProcess>
#include <QThread>
#include <QApplication>
#include <QLabel>
#include <QTabWidget>
#include "ImageThread.h"
#include <QGraphicsView>
#include "robot_map_widget.h"

SimulationTabWidget::SimulationTabWidget(QWidget* parent, QLabel* status, QLabel* cameraViewLabel, QLabel *gameImageViewLabel)
    : QWidget(parent)
    , connectionStatus(status)
    , ip_address("127.0.0.1")
    
{
    
    cameraViewWidget = new CameraViewWidget(this, cameraViewLabel);
    gameView = new GameViewWidget(this, gameImageViewLabel);
}


void SimulationTabWidget::setMapWidget(RobotMapWidget* mapWidget)
{
    robotMapWidget = mapWidget;
    cameraViewWidget->setMapWidget(robotMapWidget);
}

void SimulationTabWidget::setMapName(std::string mapName)
{
    map_name = mapName;
}


SimulationTabWidget::~SimulationTabWidget()
{
   
    
}

void SimulationTabWidget::setSimulationData(WindowData &data) {
    simData = data;
    qDebug() << "SimulationTabWidget::setSimulationData() called - data received";


}




void SimulationTabWidget::onStopServerClicked()
{
    if (m_process->state() == QProcess::Running) {
        m_process->terminate();
    }
}

void SimulationTabWidget::setGameViewWidget(GameViewWidget* view)

{
}

void SimulationTabWidget::onStartAgent()
{
    std::vector<msr::airlib::Vector3r> waypoints;
    waypoints.push_back(msr::airlib::Vector3r(100, 0, -10)); // waypoint 1: x=10, y=0, z=-10
    waypoints.push_back(msr::airlib::Vector3r(200, 10, -20)); // waypoint 2: x=20, y=10, z=-20
    waypoints.push_back(msr::airlib::Vector3r(300, 20, -30)); // waypoint 3: x=30, y=20, z=-30
    robotControl->setWaypoints(waypoints);
    robotControl->start();
}

void SimulationTabWidget::connectToServer(const std::string ip, bool isCarVehicle)
{
    try {

       
        if (isCarVehicle) {
            client = std::make_shared<msr::airlib::CarRpcLibClient>(ip_address);
            
        }
        else {
            client = std::make_shared<msr::airlib::MultirotorRpcLibClient>(ip_address);
        }
        client->confirmConnection();
        
        robotControl = std::make_shared<RobotControl>(client);

        
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
        << msg << std::endl;
    }


  

   
}

  
void SimulationTabWidget::onStartServerClicked() {

    QString program_path(simData.simulationBins + "/Windows");
    
    QString program = "";
    // Ask the user to select an .exe file using QFileDialog
    QString exeFilePath = QFileDialog::getOpenFileName(this, tr("Select .exe file"), program_path, tr("Executable Files (*.exe);;All Files (*)"));
    if (!exeFilePath.isEmpty()) {
        program = exeFilePath;
        // Use the 'program' variable containing the selected .exe file for further usage
        // For example, you can use it to execute the .exe or display the path in the UI
        qDebug() << "Selected .exe file: " << program;
    }
    QStringList arguments;
    QString sim_mode = "/settings_MultiRotor.json";
    auto exePath = simData.simulationBins + "/settings.json";

      
    arguments << "-settings=" + exePath
              << "-ResX=512"
              << "-ResY=512"
              << "-windowed"
              << "-WinX=4000"
              << "-WinY=4000"
              << "-adapter=1"
              << "-NoDisplay"
              << "-AlwaysFocus=0"
              << "-AlwaysOnTop=0"
              << "-BENCHMARK"
              << "-FPS=30"
              << "-DUMPMOVIE"
              << "-LANPLAY"
              << "-NOTEXTURESTREAMING"
              << "-USEALLAVAILABLECORES";


    // Launch the external application
    m_process = new QProcess(this);
    m_process->start(program, arguments);

    // Wait for the application to start and create its window
    if (!m_process->waitForStarted()) {
        qDebug() << "Failed to start the process.";
        return;
    }

    m_process->waitForReadyRead();

    // connect the backend server - sleep a  bit to make sure the sim is running
    std::this_thread::sleep_for(std::chrono::seconds(4));
    QDir dir;
    auto sim_dir_path = dir.absoluteFilePath(simData.simulationBins).toStdString();
    

    gameView->startDisplaying(sim_dir_path);
    cameraViewWidget->startDataVis(sim_dir_path);

    connectionStatus->setText("Connected");
    connectionStatus->setStyleSheet("QLabel { color : green; }");
}

