cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(utils)
set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(utils  ${SRCS})
target_link_libraries(utils ${OpenCV_LIBS})