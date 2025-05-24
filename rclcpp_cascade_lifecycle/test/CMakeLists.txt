ament_add_gtest(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test ${PROJECT_NAME}
  ${cascade_lifecycle_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(
  rclcpp_cascade_lifecycle_test_no_duplicates rclcpp_cascade_lifecycle_test_no_duplicates.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test_no_duplicates ${PROJECT_NAME}
  ${cascade_lifecycle_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
