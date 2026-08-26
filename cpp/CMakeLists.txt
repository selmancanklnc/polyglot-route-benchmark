cmake_minimum_required(VERSION 3.20)
project(route_optimization_cpp CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Optimization flags for Release mode
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/O2 /W4)
else()
    add_compile_options(-O3 -Wall -Wextra -pedantic)
endif()

include_directories(include)

# Main benchmark executable
add_executable(route_optimizer src/main.cpp)

# Tests
add_executable(test_physics tests/test_physics.cpp)
add_executable(test_routing tests/test_routing.cpp)

enable_testing()
add_test(NAME PhysicsTest COMMAND test_physics)
add_test(NAME RoutingTest COMMAND test_routing "${CMAKE_SOURCE_DIR}/../datasets/test_fixture.json")
