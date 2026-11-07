cmake_minimum_required(VERSION 3.5)
project(vins_fusion_ros2_d435i)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # Reuse the frozen R3 source-neutral motor contracts.  The D435i estimator
  # remains a separate package; only header-only provenance/quality policy is
  # shared so the four R4 replay modes cannot diverge semantically.
  ${CMAKE_CURRENT_SOURCE_DIR}/../vins_fusion_ros2/include
)

add_subdirectory(vins)
add_executable(vins_fusion_ros2_node src/vins_estimator.cpp src/vins_estimator_node.cpp src/visualization.cpp)
ament_target_dependencies(vins_fusion_ros2_node rclcpp rcpputils message_filters std_msgs std_srvs sensor_msgs visualization_msgs geometry_msgs nav_msgs px4_msgs tf2 tf2_ros cv_bridge image_transport)
target_link_libraries(vins_fusion_ros2_node vins_lib) 

add_executable(
  production_vins_batch
  vins/tools/production_vins_batch.cpp
)
target_link_libraries(production_vins_batch vins_lib)

add_executable(
  production_imu_gt_residual
  vins/tools/production_imu_gt_residual.cpp
)
target_link_libraries(production_imu_gt_residual vins_lib)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    stereo_track_diversity_test
    vins/test/stereo_track_diversity_test.cpp
  )
  target_include_directories(
    stereo_track_diversity_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vins/include
  )
  add_test(
    NAME stereo_track_diversity_test
    COMMAND stereo_track_diversity_test
  )
  add_executable(
    backend_candidate_lifecycle_test
    vins/test/backend_candidate_lifecycle_test.cpp
  )
  target_include_directories(
    backend_candidate_lifecycle_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vins/include
  )
  add_test(
    NAME backend_candidate_lifecycle_test
    COMMAND backend_candidate_lifecycle_test
  )
  add_executable(
    backend_state_test
    vins/test/backend_state_test.cpp
  )
  target_link_libraries(backend_state_test vins_lib)
  add_test(
    NAME backend_state_test
    COMMAND backend_state_test
  )
  add_executable(
    estimator_backend_state_adapter_test
    vins/test/estimator_backend_state_adapter_test.cpp
  )
  target_link_libraries(estimator_backend_state_adapter_test vins_lib)
  add_test(
    NAME estimator_backend_state_adapter_test
    COMMAND estimator_backend_state_adapter_test
  )
  add_executable(
    backend_candidate_store_test
    vins/test/backend_candidate_store_test.cpp
  )
  target_link_libraries(backend_candidate_store_test vins_lib)
  add_test(
    NAME backend_candidate_store_test
    COMMAND backend_candidate_store_test
  )
  add_executable(
    backend_health_metrics_test
    vins/test/backend_health_metrics_test.cpp
  )
  target_link_libraries(backend_health_metrics_test vins_lib)
  add_test(
    NAME backend_health_metrics_test
    COMMAND backend_health_metrics_test
  )
  add_executable(
    backend_approval_gate_test
    vins/test/backend_approval_gate_test.cpp
  )
  target_link_libraries(backend_approval_gate_test vins_lib)
  add_test(
    NAME backend_approval_gate_test
    COMMAND backend_approval_gate_test
  )
  add_executable(
    backend_candidate_input_buffer_test
    vins/test/backend_candidate_input_buffer_test.cpp
  )
  target_link_libraries(backend_candidate_input_buffer_test vins_lib)
  add_test(
    NAME backend_candidate_input_buffer_test
    COMMAND backend_candidate_input_buffer_test
  )
  add_executable(
    backend_recovery_checkpoint_store_test
    vins/test/backend_recovery_checkpoint_store_test.cpp
  )
  target_link_libraries(backend_recovery_checkpoint_store_test vins_lib)
  add_test(
    NAME backend_recovery_checkpoint_store_test
    COMMAND backend_recovery_checkpoint_store_test
  )
  add_executable(
    backend_recovery_candidate_selector_test
    vins/test/backend_recovery_candidate_selector_test.cpp
  )
  target_include_directories(
    backend_recovery_candidate_selector_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vins/include
  )
  add_test(
    NAME backend_recovery_candidate_selector_test
    COMMAND backend_recovery_candidate_selector_test
  )

  add_executable(
    stationary_evidence_gate_test
    vins/test/stationary_evidence_gate_test.cpp
  )
  target_link_libraries(stationary_evidence_gate_test vins_lib)
  add_test(
    NAME stationary_evidence_gate_test
    COMMAND stationary_evidence_gate_test
  )

  add_executable(
    warm_reinitialization_bridge_test
    vins/test/warm_reinitialization_bridge_test.cpp
  )
  target_link_libraries(warm_reinitialization_bridge_test vins_lib)
  add_test(
    NAME warm_reinitialization_bridge_test
    COMMAND warm_reinitialization_bridge_test
  )
  add_executable(
    motion_onset_warm_reinitialization_gate_test
    vins/test/motion_onset_warm_reinitialization_gate_test.cpp
  )
  target_link_libraries(
    motion_onset_warm_reinitialization_gate_test vins_lib
  )
  add_test(
    NAME motion_onset_warm_reinitialization_gate_test
    COMMAND motion_onset_warm_reinitialization_gate_test
  )

  add_executable(
    marginalization_scale_projection_test
    vins/test/marginalization_scale_projection_test.cpp
  )
  target_link_libraries(marginalization_scale_projection_test vins_lib)
  add_test(
    NAME marginalization_scale_projection_test
    COMMAND marginalization_scale_projection_test
  )

  add_executable(
    horizontal_velocity_factor_test
    vins/test/horizontal_velocity_factor_test.cpp
  )
  target_link_libraries(horizontal_velocity_factor_test vins_lib)
  add_test(
    NAME horizontal_velocity_factor_test
    COMMAND horizontal_velocity_factor_test
  )

  add_executable(
    rotor_drag_dynamics_factor_test
    vins/test/rotor_drag_dynamics_factor_test.cpp
  )
  target_link_libraries(rotor_drag_dynamics_factor_test vins_lib)
  add_test(
    NAME rotor_drag_dynamics_factor_test
    COMMAND rotor_drag_dynamics_factor_test
  )

  add_executable(
    gyro_bias_z_prior_factor_test
    vins/test/gyro_bias_z_prior_factor_test.cpp
  )
  target_link_libraries(gyro_bias_z_prior_factor_test vins_lib)
  add_test(
    NAME gyro_bias_z_prior_factor_test
    COMMAND gyro_bias_z_prior_factor_test
  )

  add_executable(
    pose_only_visual_factor_test
    vins/test/pose_only_visual_factor_test.cpp
  )
  target_link_libraries(pose_only_visual_factor_test vins_lib)
  add_test(
    NAME pose_only_visual_factor_test
    COMMAND pose_only_visual_factor_test
  )

endif()

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS
    vins_fusion_ros2_node
    production_vins_batch
    production_imu_gt_residual
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
