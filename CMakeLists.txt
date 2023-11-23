cmake_minimum_required(VERSION 3.10)
project(SLAM_STUDY
        VERSION 1.0.0
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(${PROJECT_NAME}_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(${PROJECT_NAME}_EXAMPLES ${${PROJECT_NAME}_PATH}/examples)

find_package(Pangolin REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/Pangolin/install/lib/cmake/Pangolin)
find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/Eigen/install/share/eigen3/cmake)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG=1)
endif()

add_subdirectory(modules)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_EXAMPLES}/plotTrajectory.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
        SLAM_STUDY_LIBS
)
