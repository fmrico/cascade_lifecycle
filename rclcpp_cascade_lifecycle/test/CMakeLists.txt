ament_add_gtest(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test.cpp
  TIMEOUT 120
)
ament_target_dependencies(rclcpp_cascade_lifecycle_test ${dependencies})
target_link_libraries(rclcpp_cascade_lifecycle_test ${PROJECT_NAME})
