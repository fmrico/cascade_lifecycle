// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>


#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"


TEST(rclcpp_cascade_lifecycle, activations_managing_basic)
{
  auto node_a = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_A");
  auto node_b = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");
  auto node_c = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_C");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_a->get_node_base_interface());
  executor.add_node(node_b->get_node_base_interface());
  executor.add_node(node_c->get_node_base_interface());
  
  node_a->add_activation("node_B");
  node_a->add_activation("node_C");

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators().empty());
  ASSERT_EQ(node_a->get_activations().size(), 2u);  
  ASSERT_TRUE(node_b->get_activations().empty());
  ASSERT_EQ(node_b->get_activators().size(), 1u);  
  ASSERT_TRUE(node_c->get_activations().empty());
  ASSERT_EQ(node_c->get_activators().size(), 1u);  
}

TEST(rclcpp_cascade_lifecycle, activations_managing_late_joining)
{
  auto node_a = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_A");
  auto node_b = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_a->get_node_base_interface());
  executor.add_node(node_b->get_node_base_interface());

  node_a->add_activation("node_B");
  node_a->add_activation("node_C");

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators().empty());
  ASSERT_EQ(node_a->get_activations().size(), 2u);  
  ASSERT_TRUE(node_b->get_activations().empty());
  ASSERT_EQ(node_b->get_activators().size(), 1u);  

  node_b = nullptr;

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 3.0) {
      executor.spin_some();
      rate.sleep();
    }
  }

  auto node_b2 = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");
  auto node_c = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_C");
  executor.add_node(node_c->get_node_base_interface());
  executor.add_node(node_b2->get_node_base_interface());

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators().empty());
  ASSERT_EQ(node_a->get_activations().size(), 2u);  
  ASSERT_TRUE(node_b2->get_activations().empty());
  ASSERT_EQ(node_b2->get_activators().size(), 1u);  
  ASSERT_TRUE(node_c->get_activations().empty());
  ASSERT_EQ(node_c->get_activators().size(), 1u); 
}

TEST(rclcpp_cascade_lifecycle, activations_chained)
{
  auto node_a = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_A");
  auto node_b = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_a->get_node_base_interface());
  executor.add_node(node_b->get_node_base_interface());

  node_a->add_activation("node_B");

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators().empty());
  ASSERT_EQ(node_a->get_activations().size(), 1u);  
  ASSERT_TRUE(node_b->get_activations().empty());
  ASSERT_EQ(node_b->get_activators().size(), 1u);  

  ASSERT_TRUE(node_a->get_activators_state().empty());
  ASSERT_FALSE(node_b->get_activators_state().empty());
  ASSERT_EQ(node_b->get_activators_state().size(), 1u);
  ASSERT_EQ(node_b->get_activators_state().begin()->first, "node_A");
  ASSERT_EQ(node_b->get_activators_state().begin()->second,
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN);
 
  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators_state().empty());
  ASSERT_FALSE(node_b->get_activators_state().empty());
  ASSERT_EQ(node_b->get_activators_state().size(), 1u);
  ASSERT_EQ(node_b->get_activators_state().begin()->first, "node_A");
  ASSERT_EQ(node_b->get_activators_state().begin()->second,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
 
  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators_state().empty());
  ASSERT_FALSE(node_b->get_activators_state().empty());
  ASSERT_EQ(node_b->get_activators_state().size(), 1u);
  ASSERT_EQ(node_b->get_activators_state().begin()->first, "node_A");
  ASSERT_EQ(node_b->get_activators_state().begin()->second,
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  
  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators_state().empty());
  ASSERT_FALSE(node_b->get_activators_state().empty());
  ASSERT_EQ(node_b->get_activators_state().size(), 1u);
  ASSERT_EQ(node_b->get_activators_state().begin()->first, "node_A");
  ASSERT_EQ(node_b->get_activators_state().begin()->second,
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST(rclcpp_cascade_lifecycle, multuple_activations_chained)
{
  auto node_a = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_A");
  auto node_b = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_B");
  auto node_c = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("node_C");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_a->get_node_base_interface());
  executor.add_node(node_b->get_node_base_interface());
  executor.add_node(node_c->get_node_base_interface());

  node_a->add_activation("node_C");
  node_b->add_activation("node_C");

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_TRUE(node_a->get_activators().empty());
  ASSERT_EQ(node_a->get_activations().size(), 1u);  
  ASSERT_TRUE(node_b->get_activators().empty());
  ASSERT_EQ(node_b->get_activations().size(), 1u);  
  ASSERT_TRUE(node_c->get_activations().empty());
  ASSERT_EQ(node_c->get_activators().size(), 2u);  

  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_b->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_b->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node_b->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node_b->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }
  
  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node_b->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }
  
  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  node_a->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  
  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_c->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  {
    rclcpp::Rate rate(10);
    auto start = node_a->now();
    while ((node_a->now() - start).seconds() < 0.5) {
      executor.spin_some();
      rate.sleep();
    }
  }

  ASSERT_EQ(node_a->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_b->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_c->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
