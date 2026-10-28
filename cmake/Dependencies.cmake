# Third-party dependencies, pinned to exact tags.
# Offline builds: pass -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=<path> (and _DOCTEST).
include(FetchContent)

# Jolt Physics (MIT). Pinned: upgrading requires re-running all physics test scenes.
FetchContent_Declare(
  JoltPhysics
  GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
  GIT_TAG        v5.3.0
  GIT_SHALLOW    TRUE
  SOURCE_SUBDIR  Build
)

set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(JPH_ENABLE_VULKAN OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
# Jolt otherwise overrides CMAKE_CXX_FLAGS_* for the whole project.
set(OVERRIDE_CXX_FLAGS OFF CACHE BOOL "" FORCE)
set(USE_ASSERTS OFF CACHE BOOL "")
set(CROSS_PLATFORM_DETERMINISTIC ${VEXORA_PHYSICS_CROSS_PLATFORM_DETERMINISM} CACHE BOOL "" FORCE)
# Portable baseline; AVX/AVX2 builds can be enabled per target platform later.
set(USE_AVX OFF CACHE BOOL "" FORCE)
set(USE_AVX2 OFF CACHE BOOL "" FORCE)
set(USE_AVX512 OFF CACHE BOOL "" FORCE)
set(USE_LZCNT OFF CACHE BOOL "" FORCE)
set(USE_TZCNT OFF CACHE BOOL "" FORCE)
set(USE_F16C OFF CACHE BOOL "" FORCE)
set(USE_FMADD OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(JoltPhysics)

if(VEXORA_BUILD_TESTS)
  FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(doctest)
endif()
