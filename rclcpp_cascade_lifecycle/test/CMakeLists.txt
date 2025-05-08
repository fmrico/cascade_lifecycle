ament_add_gtest(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test PUBLIC ${dependencies})
target_link_libraries(rclcpp_cascade_lifecycle_test ${PROJECT_NAME})
