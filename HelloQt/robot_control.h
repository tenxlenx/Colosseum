#pragma once

#include <vehicles/car/api/CarRpcLibClient.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <vector>
#include <thread>
#include <atomic>

class RobotControl
{
public:
    RobotControl(std::shared_ptr<msr::airlib::RpcLibClientBase> vehicle);
    void setWaypoints(std::vector<msr::airlib::Vector3r>& waypoints);
    void start();
    void stop();
    void update();

private:
    std::shared_ptr<msr::airlib::RpcLibClientBase> _vehicle;
    std::vector<msr::airlib::Vector3r> _waypoints;
    std::thread controller_thread;
    std::atomic<bool> run_controller;
};