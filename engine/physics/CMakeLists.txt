add_library(vexora_physics
  src/Shapes.cpp
  src/PhysicsMaterial.cpp
  src/QualityProfile.cpp
  src/SimulationClock.cpp
  src/JoltPhysicsWorld.cpp
  src/ComputeDevice.cpp
  src/CpuParticleBackend.cpp
  src/ParticleSystem.cpp
  src/SecondaryMotion.cpp
  src/Ragdoll.cpp
)
add_library(vexora::physics ALIAS vexora_physics)
target_include_directories(vexora_physics PUBLIC include)
target_link_libraries(vexora_physics PUBLIC vexora::core vexora::rig)
# Jolt is a PRIVATE dependency: no Jolt type is visible through the public headers.
target_link_libraries(vexora_physics PRIVATE Jolt)
if(VEXORA_PHYSICS_GPU)
  target_sources(vexora_physics PRIVATE src/VulkanParticleBackend.cpp)
  target_link_libraries(vexora_physics PRIVATE volk)
  target_compile_definitions(vexora_physics PRIVATE VEXORA_PHYSICS_HAS_VULKAN)
  if(NOT MSVC)
    # Vulkan code uses the `Info info{sType}` idiom: remaining members are zero-initialised.
    set_source_files_properties(src/VulkanParticleBackend.cpp PROPERTIES COMPILE_OPTIONS -Wno-missing-field-initializers)
  endif()
endif()

# The SPIR-V for shaders/particles.comp is committed in src/generated so builds need no shader
# compiler. With glslangValidator installed, `vexora_physics_shaders` regenerates it and the
# shader_spirv_up_to_date test fails if the committed copy is stale.
find_program(VEXORA_GLSLANG glslangValidator)
if(VEXORA_GLSLANG)
  set(_spv_args -V --target-env vulkan1.1 --vn kParticlesCompSpv)
  add_custom_target(vexora_physics_shaders
    COMMAND ${VEXORA_GLSLANG} ${_spv_args} -o ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/particles_comp_spv.h
            ${CMAKE_CURRENT_SOURCE_DIR}/shaders/particles.comp
    COMMENT "Regenerating particle compute SPIR-V")
  if(VEXORA_BUILD_TESTS)
    add_test(NAME shader_spirv_up_to_date
      COMMAND ${CMAKE_COMMAND} -DGLSLANG=${VEXORA_GLSLANG} "-DARGS=${_spv_args}"
              -DSRC=${CMAKE_CURRENT_SOURCE_DIR}/shaders/particles.comp
              -DCOMMITTED=${CMAKE_CURRENT_SOURCE_DIR}/src/generated/particles_comp_spv.h
              -DOUT=${CMAKE_CURRENT_BINARY_DIR}/particles_comp_spv.check.h
              -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckSpirv.cmake)
  endif()
endif()

if(WIN32)
  # Keep <windows.h> (pulled in by system headers) from defining min/max macros.
  target_compile_definitions(vexora_physics PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
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
