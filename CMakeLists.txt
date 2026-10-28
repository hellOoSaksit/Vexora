cmake_minimum_required(VERSION 3.24)
project(Vexora LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(VEXORA_BUILD_TESTS "Build physics test scenes" ON)
option(VEXORA_BUILD_BENCH "Build physics stress benchmark" ON)
# Cross-platform determinism costs ~8% performance according to Jolt's docs.
# OFF by default: we only claim same-binary determinism until a CI matrix verifies more.
option(VEXORA_PHYSICS_GPU "Build the Vulkan GPU compute backend (particles)" ON)
option(VEXORA_PHYSICS_CROSS_PLATFORM_DETERMINISM "Build Jolt with CROSS_PLATFORM_DETERMINISTIC" OFF)

# Static MSVC runtime everywhere: the app ships without needing the VC++ Redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(VEXORA_BUILD_TESTS)
  enable_testing()
endif()

include(cmake/Dependencies.cmake)

add_subdirectory(engine/core)
add_subdirectory(engine/rig)
add_subdirectory(engine/physics)
add_subdirectory(engine/simserver)
