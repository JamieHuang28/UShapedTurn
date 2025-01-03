cmake_minimum_required(VERSION 2.8.3)
project(u_shaped_turn)
set(CMAKE_CXX_STANDARD 11)
# set build type to Debug
set(CMAKE_BUILD_TYPE Debug)

set(PYBIND11_PYTHON_VERSION 3.7)
find_package(pybind11 REQUIRED)

find_package(Eigen3)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_subdirectory(thirdparty/hybrid_astar_algorithm)

add_library(lib${PROJECT_NAME} SHARED
    src/vehicle.cpp
    src/planner.cpp
    src/plan_agent.cpp
    src/neural_model.cpp
    src/path_planner.cpp
    src/drive_path.cpp
    src/stage_checker.cpp
)
target_include_directories(lib${PROJECT_NAME} PRIVATE src/)
target_include_directories(lib${PROJECT_NAME} PUBLIC include/ thirdparty/)
target_link_libraries(lib${PROJECT_NAME} PRIVATE hybrid_astar_algorithm)
set_target_properties(lib${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(${PROJECT_NAME} MODULE
    src/bindings.cc
    src/u_shaped_turn.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE thirdparty/ src/)
target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_link_libraries(${PROJECT_NAME} PRIVATE ${pybind11_LIBRARIES} hybrid_astar_algorithm lib${PROJECT_NAME})

add_executable(example_${PROJECT_NAME} src/example.cpp)

# gtest related
enable_testing()
add_subdirectory(tests)