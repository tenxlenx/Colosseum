#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QComboBox>
#include <QCommandLinkButton>
#include <QLabel>
#include <QTreeWidget>
#include <QLineEdit>
#include <QTimeEdit>
#include "simulation.h"
#include "simulation_tab_widget.h"
#include <nlohmann/json.hpp>
#include "robot_map_widget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void addCameraItem(QTreeWidget* treewidget, const QString &name);
    void getCameraData(QTreeWidget* treeWidget, std::vector<VehicleCamera>& cameraList);

    signals:
    void fileSelected(const QString& fileName);

public slots:

    
    void onSelectMapClicked();
    void onSaveConfigClicked();
    void onRecordRouteClicked();
    void onLoadRouteClicked();
    void onMenuSaveConfigClicked();
    void onMenuLoadConfigClicked();
    void onMenuNewSimulationClicked();
    
    void onNewCameraAddClicked();


private:
    Ui::MainWindow *ui;
    SimulationData simqt;
    //Tab configuration and views
    QWidget* AnlysisTab;
    QWidget* CameraViewWidget;
    RobotMapWidget* MapViewWidget;
    SimulationTabWidget* SimulationTab; // holds the simulation tab - camera views etc
    QListWidget* LoadMapWidget;
    QTabWidget* TabWidget;
    QWidget* AnalysisTab;

    nlohmann::json currentConfig;

    //Vehicle configuration
    QCheckBox* LidarCBox;
    QCheckBox* BarometerCBox;
    QCheckBox* GpsCBox;
    QCheckBox* ImuCBox;
    QCheckBox* MagnetometerCBox;

    //server configuration
    QLineEdit* ServerIpEdit;
    QLineEdit* ServerPortEdit;
    QLineEdit* ClockSpeedEdit;
    QLineEdit* SimulationBinsEdit;
    QLineEdit* RouteDbPathEdit;
    QLineEdit* ConfigPathEdit;
    QComboBox* VehicleSelectorComboBox;
    QTreeWidget* CameraParametersWidget;
    QTimeEdit *TimeOfDay;

    // buttons 
    QPushButton* StartServerButton;
    QPushButton* SelectMapButton;
    QPushButton* StopServerButton;
    QCommandLinkButton* SaveConfigButton;

    //Menu bar
    QAction* MenuRecordRoute;
    QAction* MenuLoadRoute;
    QAction* MenuSaveConfig;
    QAction* MenuLoadConfig;
    QAction* MenuNewSimulation;
    QAction* MenuNewCameraAdd;

    QAction* MenuFollowRoute;

    // Labels 
    QLabel* ConnectionStatusLabel;
    QLabel* CurrentSelectedMapLabel;

    // Image views
    QLabel* CameraViewLabel;
    QLabel* gameImageViewLabel;


    QString currentLoadedRoutePath;

    
};
//#include "mainwindow.moc"
#endif // MAINWINDOW_H
