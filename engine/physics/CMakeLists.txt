add_library(vexora_physics
  src/Types.cpp
  src/Shapes.cpp
  src/PhysicsMaterial.cpp
  src/QualityProfile.cpp
  src/SimulationClock.cpp
  src/JoltPhysicsWorld.cpp
)
add_library(vexora::physics ALIAS vexora_physics)
target_include_directories(vexora_physics PUBLIC include)
# Jolt is a PRIVATE dependency: no Jolt type is visible through the public headers.
target_link_libraries(vexora_physics PRIVATE Jolt)
if(MSVC)
  target_compile_options(vexora_physics PRIVATE /W4)
else()
  target_compile_options(vexora_physics PRIVATE -Wall -Wextra)
endif()

if(VEXORA_BUILD_TESTS)
  add_subdirectory(tests)
endif()
if(VEXORA_BUILD_BENCH)
  add_subdirectory(bench)
endif()
