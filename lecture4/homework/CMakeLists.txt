cmake_minimum_required(VERSION 3.16)
project(Lesson4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

link_directories(/opt/MVS/lib/64)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(io)
add_subdirectory(tasks)
add_subdirectory(tools)

function(add_exe name)
    add_executable(${name} src/${name}.cpp)

    target_link_libraries(${name}
        io
        auto_buff
        tools
        fmt::fmt
        ${OpenCV_LIBS}
        Eigen3::Eigen
        openvino::runtime
        MvCameraControl
        MvCameraControlWrapper
        MVGigEVisionSDK
        MVRender
        usb-1.0
    )

    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
endfunction()

add_exe(main)    
add_exe(video)  

set_target_properties(video PROPERTIES BUILD_RPATH "/opt/MVS/lib/64")

