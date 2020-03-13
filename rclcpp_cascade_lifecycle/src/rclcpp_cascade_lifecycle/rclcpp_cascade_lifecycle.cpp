// Copyright 2020 Intelligent Robotics Lab
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


#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rclcpp_cascade_lifecycle
{

using namespace std::chrono_literals;

CascadeLifecycleNode::CascadeLifecycleNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: CascadeLifecycleNode(
    node_name,
    "",
    options)
{}

CascadeLifecycleNode::CascadeLifecycleNode(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: LifecycleNode(
  node_name,
  namespace_,
  options)
{
  using std::placeholders::_1;
  
  activations_pub_ = create_publisher<cascade_lifecycle_msgs::msg::Activation>(
    "/cascade_lifecycle_activations",
    rclcpp::QoS(1000).keep_all().transient_local().reliable());

  states_pub_ = create_publisher<cascade_lifecycle_msgs::msg::State>(
    "/cascade_lifecycle_states", rclcpp::QoS(1).keep_all().transient_local().reliable());
  
  activations_sub_ = create_subscription<cascade_lifecycle_msgs::msg::Activation>(
    "/cascade_lifecycle_activations",
    rclcpp::QoS(1000).keep_all().transient_local().reliable(),
    std::bind(&CascadeLifecycleNode::activations_callback, this, _1)); 
  
  states_sub_ = create_subscription<cascade_lifecycle_msgs::msg::State>(
    "/cascade_lifecycle_states",
    rclcpp::QoS(100).keep_all().transient_local().reliable(),
    std::bind(&CascadeLifecycleNode::states_callback, this, _1)); 

  activations_pub_->on_activate();
  states_pub_->on_activate();

  register_on_configure(std::bind(
    &CascadeLifecycleNode::on_configure_internal,
    this, std::placeholders::_1));
  
  register_on_cleanup(std::bind(
    &CascadeLifecycleNode::on_cleanup_internal,
    this, std::placeholders::_1));

  register_on_shutdown(std::bind(
    &CascadeLifecycleNode::on_shutdown_internal,
    this, std::placeholders::_1));
  
  register_on_activate(std::bind(
    &CascadeLifecycleNode::on_activate_internal,
    this, std::placeholders::_1));

  register_on_deactivate(std::bind(
    &CascadeLifecycleNode::on_deactivate_internal,
    this, std::placeholders::_1));

  register_on_error(std::bind(
    &CascadeLifecycleNode::on_error_internal,
    this, std::placeholders::_1));
}

void
CascadeLifecycleNode::activations_callback(const cascade_lifecycle_msgs::msg::Activation::SharedPtr msg)
{
  switch(msg->operation_type) {
    case cascade_lifecycle_msgs::msg::Activation::ADD:
      if (msg->activation == get_name()) {
        activators_.insert(msg->activator);

        if (activators_state_.find(msg->activator) == activators_state_.end()) {
          activators_state_[msg->activator] = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
      }
      break;
    case cascade_lifecycle_msgs::msg::Activation::REMOVE:
      if (msg->activation == get_name()) {
        activators_.erase(msg->activator);

        if (activators_state_.find(msg->activator) != activators_state_.end()) {
          activators_state_.erase(msg->activator);
        }
      }
      break;   
  };
}

void
CascadeLifecycleNode::states_callback(const cascade_lifecycle_msgs::msg::State::SharedPtr msg)
{
  if (activators_state_.find(msg->node_name) != activators_state_.end() &&
    msg->node_name != get_name()) 
  {
    activators_state_[msg->node_name] = msg->state;
    update_state();
  }
}

void
CascadeLifecycleNode::add_activation(const std::string & node_name)
{
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::ADD;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.insert(node_name);

    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to set an auto activation");
  }
}

void
CascadeLifecycleNode::remove_activation(const std::string & node_name)
{
  if (node_name != get_name()) {
    cascade_lifecycle_msgs::msg::Activation msg;
    msg.operation_type = cascade_lifecycle_msgs::msg::Activation::REMOVE;
    msg.activator = get_name();
    msg.activation = node_name;

    activations_.erase(node_name);

    activations_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "Trying to remove an auto activation");
  }
}

void
CascadeLifecycleNode::clear_activation()
{
  for (const auto & activation : activations_) {
    remove_activation(activation);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_configure_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_configure(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_cleanup_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_cleanup(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_shutdown_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_shutdown(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_activate_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_activate(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    msg.node_name = get_name();
 
    states_pub_->publish(msg);
  }

  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_deactivate_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_deactivate(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CascadeLifecycleNode::on_error_internal(
  const rclcpp_lifecycle::State & previous_state)
{
  cascade_lifecycle_msgs::msg::State msg;

  auto ret = on_error(previous_state);

  if (ret == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    cascade_lifecycle_msgs::msg::State msg;
    msg.state = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    msg.node_name = get_name();

    states_pub_->publish(msg);
  }

  return ret;
}

void 
CascadeLifecycleNode::update_state()
{
  bool parent_inactive = false;
  bool parent_active = false;

  for (const auto & activator : activators_state_) {
    parent_inactive = parent_inactive ||
      activator.second == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    parent_active = parent_active ||
      activator.second == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  switch(get_current_state().id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
      if (parent_active || parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
      break;

     case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      if (parent_active || parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
      break;   
    
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      if (parent_active) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      }
      break;
    
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      if (!parent_active && parent_inactive) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
      break;
    
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      break;
  }
}

}  // namespace rclcpp_cascade_lifecycle