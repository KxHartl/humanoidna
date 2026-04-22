cmake_minimum_required(VERSION 3.16)
project(rgb_pcd_object_analysis LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(PCL REQUIRED COMPONENTS common io visualization segmentation search features filters)

add_executable(rgb_pcd_object_analysis src/main.cpp)

target_include_directories(rgb_pcd_object_analysis PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

if(PCL_DEFINITIONS)
    add_definitions(${PCL_DEFINITIONS})
endif()

target_link_libraries(rgb_pcd_object_analysis PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
