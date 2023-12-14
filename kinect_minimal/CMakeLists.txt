cmake_minimum_required(VERSION 2.8.12.1)

project(kinect)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(freenect2 REQUIRED)

include_directories(
    "/usr/include/libusb-1.0/"
    ${freenect2_INCLUDE_DIR}
    )

add_executable(main ./main.cpp)

target_link_libraries(main ${freenect2_LIBRARIES})