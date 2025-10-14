cmake_minimum_required(VERSION 3.16)
set(OpenVINO_DIR "/usr/lib/cmake/openvino2024.6.0")  # 修改为你的OpenVINO安装路径
find_package(OpenVINO REQUIRED)

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    yolo11_buff.cpp
    buff_solver.cpp
)
target_include_directories(auto_buff PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(auto_buff openvino::runtime ${CERES_LIBRARIES})