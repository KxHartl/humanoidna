cmake_minimum_required(VERSION 3.10)
project(Realsense_humanoidna)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(realsense2 QUIET)
if(NOT realsense2_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(REALSENSE2 REQUIRED realsense2)
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

if(NOT realsense2_FOUND)
    include_directories(${REALSENSE2_INCLUDE_DIRS})
    link_directories(${REALSENSE2_LIBRARY_DIRS})
    add_definitions(${REALSENSE2_CFLAGS_OTHER})
endif()

add_executable(Realsense_humanoidna src/realsense_humanoidna.cpp)

target_link_libraries(Realsense_humanoidna
    PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

if(realsense2_FOUND)
    target_link_libraries(Realsense_humanoidna PRIVATE realsense2::realsense2)
else()
    target_link_libraries(Realsense_humanoidna PRIVATE ${REALSENSE2_LIBRARIES})
endif()
