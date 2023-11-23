cmake_minimum_required(VERSION 3.10)
project(SLAM_STUDY_LIBS
        VERSION 1.0.0
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(${PROJECT_NAME}_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(${PROJECT_NAME}_SOURCE_PATH ${${PROJECT_NAME}_PATH}/src)

file(GLOB ${PROJECT_NAME}_SOURCE_FILES
        "${${PROJECT_NAME}_SOURCE_PATH}/*.cpp"
)

add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_SOURCE_FILES}
)

find_package(Eigen3 REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/Eigen/install/share/eigen3/cmake)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 Found! -${Eigen3_DIR}")
    set(Eigen3_LIBS Eigen3::Eigen)
endif()

find_package(Pangolin REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/Pangolin/install/lib/cmake/Pangolin)
if(Pangolin_FOUND)
    message(STATUS "Pangolin Found! -${Pangolin_DIR}")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${Eigen3_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${Eigen3_LIBS}
        ${Pangolin_LIBRARIES}
)