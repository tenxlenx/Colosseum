
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.25.0)
set(CMAKE_CUDA_ARCHITECTURES 89)
project(HelloQt LANGUAGES CXX CUDA)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Qt/6.5.1/msvc2019_64")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
if(MSVC)
    # Use multiple processors for compilation
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
endif()




set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)



LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()
IncludeEigen()
SetupConsoleBuild()

find_package(Qt6  REQUIRED)
find_package(Qt6Core  REQUIRED)
find_package(Qt6Gui  REQUIRED)
find_package(Qt6Widgets  REQUIRED)
find_package(Qt63DCore  REQUIRED)
find_package(Qt63DRender  REQUIRED)
find_package(Qt63DExtras  REQUIRED)
find_package(Qt63DInput  REQUIRED)
find_package(Qt63DLogic  REQUIRED)
find_package(Qt63DAnimation  REQUIRED)
find_package(Qt6 COMPONENTS Core Gui 3DCore 3DRender 3DExtras 3DInput REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(PCL CONFIG REQUIRED)



## Specify additional locations of header files
include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/../../HelloQt
  ${AIRSIM_ROOT}/HelloQt
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}

)


set(QtInstallDir "C:/Qt/6.5.1/msvc2019_64")

# Copy the required Qt DLLs to the build directory
file(GLOB_RECURSE QtDLLs "${QtInstallDir}/bin/*.dll")
file(COPY ${QtDLLs} DESTINATION ${CMAKE_BINARY_DIR})

set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cpp" "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cu")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})

#add_dependencies(${PROJECT_NAME} AirLib)

set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES "50;72;89")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}  CUDA::cudart
        CUDA::cublas
        CUDA::cublasLt
        CUDA::cufft
        CUDA::cusolver
        CUDA::cuda_driver)
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::3DCore
    Qt6::3DRender
    Qt6::3DExtras
    Qt6::3DInput
    Qt6::3DLogic)
	

   

