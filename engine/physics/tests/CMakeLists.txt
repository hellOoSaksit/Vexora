add_executable(vexora_physics_tests
  main.cpp
  test_clock_and_math.cpp
  test_validation.cpp
  scene01_rigid_body_falling.cpp
  scene02_stacking.cpp
  scene03_joints.cpp
  scene04_ragdoll.cpp
  scene06_secondary_motion.cpp
  scene09_particles.cpp
  scene10_determinism_snapshot.cpp
  test_transform_sync_and_events.cpp
)
target_link_libraries(vexora_physics_tests PRIVATE vexora::physics doctest::doctest)
add_test(NAME vexora_physics_tests COMMAND vexora_physics_tests)
