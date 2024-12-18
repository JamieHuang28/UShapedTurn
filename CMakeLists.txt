cmake_minimum_required(VERSION 2.8.3)
project(u_shaped_turn)
set(CMAKE_CXX_STANDARD 11)
# set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

set(PYBIND11_PYTHON_VERSION 3.7)
find_package(pybind11 REQUIRED)

find_package(Eigen3)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

pybind11_add_module(${PROJECT_NAME} MODULE
    src/bindings.cc
    src/u_shaped_turn.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${pybind11_LIBRARIES})

add_executable(example src/example.cpp)