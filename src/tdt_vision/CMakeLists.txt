cmake_minimum_required(VERSION 3.8)
project(tdt_vision)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(  
  "detect/include"
  "resolve/include"
  "calibrate/include"
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(utils)
include_directories(${PROJECT_SOURCE_DIR}/utils/include)

set(RADAR_DETECT "radar_detect")
file(GLOB_RECURSE RADAR_DETECT_SRCS "detect/src/*.cpp")
ament_auto_add_library(${RADAR_DETECT} SHARED
    ${RADAR_DETECT_SRCS}
)
find_package(CUDA)
include(${PROJECT_SOURCE_DIR}/cmake/modules/libnvinferConfig.cmake)
message("Using Cuda Version: ${CUDA_VERSION} and TensorRT Version: ${nvinfer_VERSION}")
add_definitions(-D use_tensorrt)
enable_language(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${nvinfer_INCLUDE_DIRS})

add_subdirectory(infer)
include_directories(${PROJECT_SOURCE_DIR}/infer/include)

target_link_libraries(${RADAR_DETECT}
    utils
    infer 
    ${nvinfer_LIBRARY} 
    cudart 
    nvonnxparser 
    fmt 
    ${PCL_LIBRARIES}
    )
rclcpp_components_register_node(${RADAR_DETECT}
    PLUGIN "tdt_radar::Detect"
    EXECUTABLE ${RADAR_DETECT}_node
)

set(RADAR_RESOLVE "radar_resolve")
file(GLOB_RECURSE RADAR_RESOLVE_SRCS "resolve/src/*.cpp")

ament_auto_add_library(${RADAR_RESOLVE} SHARED
    ${RADAR_RESOLVE_SRCS}
)
rclcpp_components_register_node(${RADAR_RESOLVE}
    PLUGIN "tdt_radar::Resolve"
    EXECUTABLE ${RADAR_RESOLVE}_node
)

set(RADAR_CALIBRATE "radar_calibrate")
file(GLOB_RECURSE RADAR_CALIBRATE_SRCS "calibrate/src/*.cpp")
ament_auto_add_library(${RADAR_CALIBRATE} SHARED
    ${RADAR_CALIBRATE_SRCS}
)
rclcpp_components_register_node(${RADAR_CALIBRATE}
    PLUGIN "tdt_radar::Calibrate"
    EXECUTABLE ${RADAR_CALIBRATE}_node
)


ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
