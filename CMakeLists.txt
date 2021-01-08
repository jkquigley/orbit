project(orbit)
set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE Release)



find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_lib src/Body.cpp src/System.cpp src/Parser.cpp)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PROJECT_NAME}_lib Eigen3::Eigen ${YAML_CPP_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
