cmake_minimum_required(VERSION 3.25.0)
set(CMAKE_CXX_STANDARD 17)
project(AirSim)

add_subdirectory("rpclib_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("AirLibUnitTests")
add_subdirectory("HelloDrone")
add_subdirectory("HelloSpawnedDrones")
add_subdirectory("HelloCar")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")


