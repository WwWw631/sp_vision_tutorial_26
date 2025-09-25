cmake_minimum_required(VERSION 3.10)
project(lecture_2)

include_directories(part_1)
add_executable(cpp_basics part_1/cpp_basics.cpp part_1/tools.cpp)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(split_channels part_2/split_channels.cpp )
target_link_libraries(split_channels ${OpenCV_LIBS})

add_executable(lightbar_detection part_3/lightbar_detection.cpp)
target_link_libraries(lightbar_detection ${OpenCV_LIBS})

#fmt_test.cpp
cmake_minimum_required(VERSION 3.10)
project(homework)

find_package(fmt CONFIG REQUIRED)

add_executable(fmt_test homework/fmt_test.cpp)
target_link_libraries(fmt_test PRIVATE fmt::fmt)

#image_resize
cmake_minimum_required(VERSION 3.10)
project(homework)

set(CMAKE_CXX_STANDARD 17)

# 添加 fmt 和 OpenCV
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

add_executable(image_resize homework/main.cpp homework/tools.cpp)

target_link_libraries(image_resize ${OpenCV_LIBS} fmt::fmt)
