cmake_minimum_required(VERSION 3.25.0)
set(CMAKE_CUDA_ARCHITECTURES 86)
project(HelloCar LANGUAGES CXX CUDA)


LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

SetupConsoleBuild()
find_package(protobuf CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(CUDAToolkit REQUIRED)

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloCar
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
)

AddExecutableSource()

target_link_libraries(${PROJECT_NAME}  CUDA::cudart
        CUDA::cublas
        CUDA::cublasLt
        CUDA::cufft
        CUDA::cusolver
        CUDA::cuda_driver)
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
