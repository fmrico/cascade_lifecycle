// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Its own binary: it shuts ROS down

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

using namespace std::chrono_literals;  // NOLINT

// Ctrl-C while the nodes spin: the state timer used to ask for the node names with the
// context already shut down, and the exception aborted the process
TEST(rclcpp_cascade_lifecycle, shutdown_while_spinning)
{
  // Different moments of the 500 ms state timer
  for (auto delay : {450ms, 520ms, 610ms, 700ms, 790ms, 880ms, 970ms, 1060ms, 1150ms, 1240ms}) {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    auto parent = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("parent");
    auto child = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("child");
    child->add_activation("parent");

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(parent->get_node_base_interface());
    exe.add_node(child->get_node_base_interface());
    std::atomic<bool> finish {false};
    // Keeps spinning after shutdown, as an executor does until it notices
    std::thread spinner([&]() {
        while (!finish) {exe.spin_once(10ms);}
      });

    parent->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    parent->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    std::this_thread::sleep_for(delay);

    rclcpp::shutdown();
    // Several timer periods with the context gone
    std::this_thread::sleep_for(1200ms);
    finish = true;
    spinner.join();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
