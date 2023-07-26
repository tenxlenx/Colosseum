#include "robot_control.h"
#include "robot_control.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <fstream>
#include <iostream>
#include <iostream>
#include <sstream>
#include <string>
#include <string>
#include <thread>
#include <vector>
#include <vector>
#include "common/Common.hpp"
#include "robot_control.h"

RobotControl::RobotControl(std::shared_ptr<msr::airlib::RpcLibClientBase> vehicle)
    : _vehicle(vehicle), run_controller(true)
{
    bool use_drone = true; // set this to false to use a car

    if (use_drone) {
        _vehicle = std::make_shared<msr::airlib::MultirotorRpcLibClient>();
    }
    else {
        _vehicle = std::make_shared<msr::airlib::CarRpcLibClient>();
    }

    // Connect to the AirSim simulator
    _vehicle->confirmConnection();

   
}

void RobotControl::setWaypoints(std::vector<msr::airlib::Vector3r>& waypoints)
{
    _waypoints = waypoints;
}

void RobotControl::start()
{
    // Start the waypoint controller in a separate thread
    controller_thread = std::thread([&]() {
        while (run_controller) {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

void RobotControl::stop()
{
    // Stop the waypoint controller thread
    run_controller = false;
    if (controller_thread.joinable()) {
        controller_thread.join();
    }
}


void RobotControl::update()
{
    // Carrot stick following
    float lookahead = -1;
    float adaptive_lookahead = 1.0;
    float speed = 5.0; // speed to move between waypoints in m/s
    float timeout_sec = 3.0; // timeout in seconds
    msr::airlib::DrivetrainType drivetrain = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    msr::airlib::YawMode yaw_mode(true, 0); // auto yaw mode

    if (auto* drone = dynamic_cast<msr::airlib::MultirotorApiBase*>(_vehicle.get())) {
        drone->moveOnPath(_waypoints, speed, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
    }
    else if (auto* car = dynamic_cast<msr::airlib::CarApiBase*>(_vehicle.get())) {
        // Implement car-specific waypoint following here
    }
    // Add more else if blocks here for other types of vehicles
}