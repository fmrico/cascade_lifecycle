ament_add_gtest(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test ${PROJECT_NAME} ${dependencies})

ament_add_gtest(
  rclcpp_cascade_lifecycle_test_no_duplicates rclcpp_cascade_lifecycle_test_no_duplicates.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_test_no_duplicates ${dependencies} ${PROJECT_NAME})

ament_add_gtest(
  rclcpp_cascade_lifecycle_shutdown_test rclcpp_cascade_lifecycle_shutdown_test.cpp
  TIMEOUT 120
)
target_link_libraries(rclcpp_cascade_lifecycle_shutdown_test ${dependencies} ${PROJECT_NAME})

# Tests discover only nodes on this host: parallel jobs do not mix
set_tests_properties(
  rclcpp_cascade_lifecycle_test rclcpp_cascade_lifecycle_test_no_duplicates
  rclcpp_cascade_lifecycle_shutdown_test
  PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
