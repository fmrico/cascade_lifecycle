ament_add_gtest(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test ${PROJECT_NAME} ${cascade_lifecycle_msgs_TARGETS} ${lifecycle_msgs_TARGETS} rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
