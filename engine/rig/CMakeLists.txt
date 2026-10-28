# Rig data shared by animation, physics and export: skeleton, poses, skinning, clips.
add_library(vexora_rig src/Skeleton.cpp src/Animation.cpp src/Skinning.cpp src/GltfImport.cpp)
add_library(vexora::rig ALIAS vexora_rig)
target_include_directories(vexora_rig PUBLIC include)
target_link_libraries(vexora_rig PUBLIC vexora::core)
target_include_directories(vexora_rig PRIVATE ${cgltf_SOURCE_DIR})
# cgltf is C-style code; keep its warnings out of our build.
set_source_files_properties(src/GltfImport.cpp PROPERTIES COMPILE_DEFINITIONS "_CRT_SECURE_NO_WARNINGS")
if(MSVC)
  target_compile_options(vexora_rig PRIVATE /W4)
else()
  target_compile_options(vexora_rig PRIVATE -Wall -Wextra)
endif()

if(VEXORA_BUILD_TESTS)
  add_executable(vexora_rig_tests tests/test_gltf_import.cpp)
  target_link_libraries(vexora_rig_tests PRIVATE vexora::rig doctest::doctest)
  target_include_directories(vexora_rig_tests PRIVATE ${cgltf_SOURCE_DIR})
  target_compile_definitions(vexora_rig_tests PRIVATE
    VEXORA_SAMPLES_DIR="${PROJECT_SOURCE_DIR}/assets/samples"
    VEXORA_TEST_TMP="${CMAKE_CURRENT_BINARY_DIR}")
  add_test(NAME vexora_rig_tests COMMAND vexora_rig_tests)
endif()
