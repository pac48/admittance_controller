// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Denis Stogl, Andy Zelenak

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_

#include "admittance_controller/admittance_rule.hpp"

#include "angles/angles.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/utilities.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace admittance_controller
{

controller_interface::return_type AdmittanceRule::configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
  clock_ = node->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize variables used in the update loop
  measured_wrench_.header.frame_id = parameters_.sensor_frame_;

  // The variables represent transformation within the same frame
  relative_admittance_pose_ik_base_frame_.header.frame_id = parameters_.ik_base_frame_;
  relative_admittance_pose_ik_base_frame_.child_frame_id = parameters_.ik_base_frame_;
  relative_admittance_pose_control_frame_.header.frame_id = parameters_.control_frame_;
  relative_admittance_pose_control_frame_.child_frame_id = parameters_.control_frame_;

  // The variables represent transformation within the same frame
  admittance_velocity_ik_base_frame_.header.frame_id = parameters_.ik_base_frame_;
  admittance_velocity_ik_base_frame_.child_frame_id = parameters_.ik_base_frame_;
  admittance_velocity_control_frame_.header.frame_id = parameters_.control_frame_;
  admittance_velocity_control_frame_.child_frame_id = parameters_.control_frame_;

  sum_of_admittance_displacements_.header.frame_id = parameters_.ik_base_frame_;

  reference_joint_deltas_vec_.resize(6, 0.0);
  reference_deltas_vec_ik_base_.reserve(6);
  // The variables represent transformation within the same frame
  reference_deltas_ik_base_.header.frame_id = parameters_.ik_base_frame_;
  reference_deltas_ik_base_.child_frame_id = parameters_.ik_base_frame_;

  identity_transform_.transform.rotation.w = 1;
  identity_transform_.header.frame_id = parameters_.ik_base_frame_;

  relative_desired_joint_state_vec_.resize(6, 0.0);

  admittance_rule_calculated_values_.positions.resize(6, 0.0);
  admittance_rule_calculated_values_.velocities.resize(6, 0.0);
  admittance_rule_calculated_values_.accelerations.resize(6, 0.0);
  admittance_rule_calculated_values_.effort.resize(6, 0.0);

  // Load the differential IK plugin
  if (!parameters_.ik_plugin_name_.empty())
  {
    try
    {
      // TODO(destogl): add "ik_interface" into separate package and then rename the package in
      // the next line from "admittance_controller" to "ik_base_plugin"
      ik_loader_ = std::make_shared<pluginlib::ClassLoader<ik_interface::IKBaseClass>>(
        "ik_interface", "ik_interface::IKBaseClass");
      ik_ = std::unique_ptr<ik_interface::IKBaseClass>(
        ik_loader_->createUnmanagedInstance(parameters_.ik_plugin_name_));
      if (!ik_->initialize(node, parameters_.ik_group_name_))
      {
        return controller_interface::return_type::ERROR;
      }
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "Exception while loading the IK plugin '%s': '%s'",
                   parameters_.ik_plugin_name_.c_str(), ex.what());
      return controller_interface::return_type::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"), "A differential IK plugin name was not specified in the config file.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::reset()
{
  measured_wrench_ik_base_frame_arr_.fill(0.0);
  reference_pose_arr_.fill(0.0);
  current_pose_arr_.fill(0.0);
  admittance_velocity_arr_.fill(0.0);
  sum_of_admittance_displacements_arr_.fill(0.0);

  get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);
  reference_pose_from_joint_deltas_ik_base_frame_ = current_pose_ik_base_frame_;

  // "Open-loop" controller uses old desired pose as current pose: current_pose(K) = desired_pose(K-1)
  // Therefore desired pose has to be set before calling *update*-method
  if (parameters_.open_loop_control_) {
    get_pose_of_control_frame_in_base_frame(admittance_pose_ik_base_frame_);
    convert_message_to_array(admittance_pose_ik_base_frame_, admittance_pose_ik_base_frame_arr_);
  }

  return controller_interface::return_type::OK;
}

// Update with target Cartesian pose - the main update method!
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const geometry_msgs::msg::PoseStamped & reference_pose,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // Convert inputs to ik_base frame (assumed stationary)
  transform_to_ik_base_frame(reference_pose, reference_pose_ik_base_frame_);

  std::array<double, 6> pose_error;
  geometry_msgs::msg::TransformStamped pose_error_pose;
  pose_error_pose.header.frame_id = parameters_.ik_base_frame_;
  pose_error_pose.child_frame_id = parameters_.control_frame_;

  if (!parameters_.open_loop_control_ || true) {
    get_pose_of_control_frame_in_base_frame(current_pose_ik_base_frame_);

    // Convert all data to arrays for simpler calculation
    transform_to_control_frame(reference_pose_ik_base_frame_, reference_pose_control_frame_);
    convert_message_to_array(reference_pose_control_frame_, reference_pose_arr_);
    transform_to_control_frame(current_pose_ik_base_frame_, current_pose_control_frame_);
    convert_message_to_array(current_pose_control_frame_, current_pose_arr_);

    for (auto i = 0u; i < 6; ++i) {
      pose_error[i] = current_pose_arr_[i] - reference_pose_arr_[i];

      if (i >= 3) {
        pose_error[i] = angles::normalize_angle(pose_error[i]);
      }
      if (std::fabs(pose_error[i]) < POSE_ERROR_EPSILON) {
        pose_error[i] = 0.0;
      }
    }

  } else {
    // In open-loop mode, assume the user's requested pose was exactly achieved
    // TODO(destogl): This will maybe now work when no feed-forward is used
    current_pose_ik_base_frame_ =
      reference_pose_ik_base_frame_;  // FIXME: current pose is actually output of the admittance and not reference -> then pose error is again current - reference

//     current_pose_ik_base_frame_ = admittance_pose_ik_base_frame_;  // TEST

    // Sum admittance displacements (in ik_base_frame) from the previous relative poses - this is
    // needed because we are using feed-forward term in L221 when using joints
    // This could be probably remove to that we calculate admittance_desired - reference to get the pose_error - cool this could be done above!!!! very cool, we need to try this...
    // Feedforward term is current - reference
    for (auto i = 0u; i < 6; ++i)
    {
      sum_of_admittance_displacements_arr_[i] += relative_admittance_pose_arr_[i];
    }

    // Transform sum of admittance displacements to control frame
    convert_array_to_message(sum_of_admittance_displacements_arr_, sum_of_admittance_displacements_);
    transform_relative_to_control_frame(sum_of_admittance_displacements_, sum_of_admittance_displacements_control_frame_);
    convert_message_to_array(sum_of_admittance_displacements_control_frame_, pose_error);
  }

  process_wrench_measurements(measured_wrench);

  // Transform internal state to updated control frame - could be changed since the last update
  transform_relative_to_control_frame(
    admittance_velocity_ik_base_frame_, admittance_velocity_control_frame_);
  convert_message_to_array(admittance_velocity_control_frame_, admittance_velocity_arr_);

  // Calculate admittance rule in the control frame
  calculate_admittance_rule(
    measured_wrench_ik_base_frame_arr_, pose_error, period, relative_admittance_pose_arr_);

  // Transform internal states from current "control" frame to "ik base" frame
  // Do clean conversion to the goal pose using transform and not messing with Euler angles
  convert_array_to_message(relative_admittance_pose_arr_, relative_admittance_pose_control_frame_);
  transform_relative_to_ik_base_frame(
    relative_admittance_pose_control_frame_, relative_admittance_pose_ik_base_frame_);
  convert_message_to_array(relative_admittance_pose_ik_base_frame_, relative_admittance_pose_arr_);

  convert_array_to_message(admittance_velocity_arr_, admittance_velocity_control_frame_);
  transform_relative_to_ik_base_frame(
    admittance_velocity_control_frame_, admittance_velocity_ik_base_frame_);

  // Add deltas to previously-desired pose to get the next desired pose
  tf2::doTransform(current_pose_ik_base_frame_, admittance_pose_ik_base_frame_,
                   relative_admittance_pose_ik_base_frame_);

  return calculate_desired_joint_state(current_joint_state, relative_admittance_pose_arr_,
                                       period, desired_joint_state);
}

// Update from reference joint states
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const trajectory_msgs::msg::JointTrajectoryPoint & reference_joint_state,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
    auto num_joints_ = current_joint_state.positions.size();
  desired_joint_state.positions.assign(num_joints_, 0.0);
    desired_joint_state.velocities.assign(num_joints_, 0.0);
    if (reference_joint_state.positions.empty())
    {
      desired_joint_state.accelerations.assign(num_joints_, 0.0);
    }
    else
    {
        desired_joint_state.positions = reference_joint_state.positions;
    }

  reference_joint_deltas_vec_.assign(reference_joint_deltas_vec_.size(), 0.0);

  // Calculate joint_deltas only when feed-forward is needed, i.e., trajectory is valid
  // If there are no positions, expect velocities
  if (reference_joint_state.positions.empty())
  {
    for (size_t index = 0; index < reference_joint_state.velocities.size(); ++index)
    {
      reference_joint_deltas_vec_[index] =
        reference_joint_state.velocities[index] * .1;//period.seconds();
    }
  }
  else
  {
    for (size_t index = 0; index < reference_joint_state.positions.size(); ++index)
    {
      reference_joint_deltas_vec_[index] = angles::shortest_angular_distance(
        current_joint_state.positions[index], reference_joint_state.positions[index]);
    }
  }


  // Get feed-forward cartesian deltas in the ik_base frame.
  // Since ik_base is MoveIt's working frame, the transform is identity.
  ik_->update_robot_state(current_joint_state);
  if (!ik_->convert_joint_deltas_to_cartesian_deltas(
    reference_joint_deltas_vec_, identity_transform_, reference_deltas_vec_ik_base_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                 "Conversion of joint deltas to Cartesian deltas failed. Sending current joint"
                 " values to the robot.");
    desired_joint_state = current_joint_state;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  convert_array_to_message(reference_deltas_vec_ik_base_, reference_deltas_ik_base_);

    reference_pose_from_joint_deltas_ik_base_frame_ = geometry_msgs::msg::PoseStamped(); // reset to zero
  // Add deltas to previously-desired pose to get the next desired pose
  tf2::doTransform(reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_deltas_ik_base_);

  update(current_joint_state, measured_wrench, reference_pose_from_joint_deltas_ik_base_frame_,
         period, desired_joint_state);

  return controller_interface::return_type::OK;
}

// Update from reference joint deltas
controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const geometry_msgs::msg::Wrench & measured_wrench,
  const std::array<double, 6> & reference_joint_deltas,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state)
{
  reference_joint_deltas_vec_.assign(reference_joint_deltas.begin(), reference_joint_deltas.end());

  // Get feed-forward cartesian deltas in the ik_base frame.
  // Since ik_base is MoveIt's working frame, the transform is identity.
  ik_->update_robot_state(current_joint_state);
  if (!ik_->convert_joint_deltas_to_cartesian_deltas(
      reference_joint_deltas_vec_, identity_transform_, reference_deltas_vec_ik_base_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AdmittanceRule"),
                 "Conversion of joint deltas to Cartesian deltas failed. Sending current joint"
                 " values to the robot.");
    desired_joint_state = current_joint_state;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  convert_array_to_message(reference_deltas_vec_ik_base_, reference_deltas_ik_base_);

  // Add deltas to previously-desired pose to get the next desired pose
  tf2::doTransform(reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_pose_from_joint_deltas_ik_base_frame_,
                   reference_deltas_ik_base_);

  update(current_joint_state, measured_wrench, reference_pose_from_joint_deltas_ik_base_frame_,
         period, desired_joint_state);

  // TODO(destogl): this is moved to admittance controller
  //   for (auto i = 0u; i < desired_joint_state.positions.size(); ++i) {  // TEST
  //     desired_joint_state.positions[i] += reference_joint_deltas[i];
  //     desired_joint_state.velocities[i] += reference_joint_deltas[i] / period.seconds();
  //   }

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_state*/,
  const geometry_msgs::msg::Wrench & /*measured_wrench*/,
  const geometry_msgs::msg::PoseStamped & /*reference_pose*/,
  const geometry_msgs::msg::WrenchStamped & /*reference_force*/,
  const rclcpp::Duration & /*period*/,
  trajectory_msgs::msg::JointTrajectoryPoint & /*desired_joint_state*/
)
{
  // TODO(destogl): Implement this update
  //  transform_message_to_ik_base_frame(**input_force_cmd, force_cmd_ctrl_frame_);
  // TODO(destogl) reuse things from other update

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_controller_state(
  control_msgs::msg::AdmittanceControllerState & state_message)
{
  //   state_message.input_wrench_control_frame = reference_wrench_control_frame_;
  state_message.input_pose_control_frame = reference_pose_ik_base_frame_;
  state_message.measured_wrench = measured_wrench_;
  state_message.measured_wrench_filtered = measured_wrench_filtered_;
  state_message.measured_wrench_control_frame = measured_wrench_ik_base_frame_;

  state_message.admittance_rule_calculated_values = admittance_rule_calculated_values_;

  state_message.current_pose = current_pose_ik_base_frame_;
  state_message.desired_pose = admittance_pose_ik_base_frame_;
  // TODO(destogl): Enable this field for debugging.
//   state_message.relative_admittance = sum_of_admittance_displacements_;
  state_message.relative_desired_pose = relative_admittance_pose_ik_base_frame_;

  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceRule::get_pose_of_control_frame_in_base_frame(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(parameters_.ik_base_frame_, parameters_.control_frame_, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation= transform.transform.rotation;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      rclcpp::get_logger("AdmittanceRule"), *clock_, 5000, "%s", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void AdmittanceRule::process_wrench_measurements(
  const geometry_msgs::msg::Wrench & measured_wrench
)
{
  measured_wrench_.wrench = measured_wrench;
  filter_chain_->update(measured_wrench_, measured_wrench_filtered_);

  // TODO(destogl): rename this variables...
  transform_to_ik_base_frame(measured_wrench_filtered_, measured_wrench_ik_base_frame_);
  transform_to_control_frame(measured_wrench_filtered_, measured_wrench_ik_base_frame_);
  convert_message_to_array(measured_wrench_ik_base_frame_, measured_wrench_ik_base_frame_arr_);

  // TODO(destogl): optimize this checks!
  // If at least one measured force is nan set all to 0
  if (std::find_if(measured_wrench_ik_base_frame_arr_.begin(),
    measured_wrench_ik_base_frame_arr_.end(),
    [](const auto value){ return std::isnan(value); }) != measured_wrench_ik_base_frame_arr_.end())
  {
    measured_wrench_ik_base_frame_arr_.fill(0.0);
  }

  // If a force or a torque is very small set it to 0
  for (auto i = 0u; i < measured_wrench_ik_base_frame_arr_.size(); ++i) {
    if (std::fabs(measured_wrench_ik_base_frame_arr_[i]) < WRENCH_EPSILON) {
      measured_wrench_ik_base_frame_arr_[i] = 0.0;
    }
  }
}

void AdmittanceRule::calculate_admittance_rule(
  const std::array<double, 6> & measured_wrench,
  const std::array<double, 6> & pose_error,
  const rclcpp::Duration & period,
  std::array<double, 6> & desired_relative_pose
)
{
  // Compute admittance control law: F = M*a + D*v + S*(x - x_d)
  for (size_t axis = 0; axis < 6; ++axis)
  {
    if (parameters_.selected_axes_[axis])
    {
      // TODO(destogl): check if velocity is measured from hardware
      const double admittance_acceleration = (1 / parameters_.mass_[axis]) * (measured_wrench[axis] -
                                                   parameters_.damping_[axis] * admittance_velocity_arr_[axis] -
                                                   parameters_.stiffness_[axis] * pose_error[axis]);

      admittance_velocity_arr_[axis] += admittance_acceleration * 0.1;//period.nanoseconds()

      // Calculate position
      desired_relative_pose[axis] = admittance_velocity_arr_[axis] * 0.1;
      if (std::fabs(desired_relative_pose[axis]) < POSE_EPSILON)
      {
        desired_relative_pose[axis] = 0.0;
      }

      // Store data for publishing to state variable
      admittance_rule_calculated_values_.positions[axis] = pose_error[axis];
      admittance_rule_calculated_values_.velocities[axis] = admittance_velocity_arr_[axis];
      admittance_rule_calculated_values_.accelerations[axis] = admittance_acceleration;
      admittance_rule_calculated_values_.effort[axis] = measured_wrench[axis];
    }
  }
}

controller_interface::return_type AdmittanceRule::calculate_desired_joint_state(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state,
  const std::array<double, 6> & relative_pose,
  const rclcpp::Duration & period,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_state
)
{
  // Since ik_base is MoveIt's working frame, the transform is identity.
  identity_transform_.header.frame_id = parameters_.ik_base_frame_;

  // Use Jacobian-based IK
  std::vector<double> relative_admittance_pose_vec(relative_pose.begin(), relative_pose.end());
  ik_->update_robot_state(current_joint_state);
  if (ik_->convert_cartesian_deltas_to_joint_deltas(
        relative_admittance_pose_vec, identity_transform_, relative_desired_joint_state_vec_))
  {
    for (auto i = 0u; i < desired_joint_state.positions.size(); ++i)
    {
      desired_joint_state.positions[i] =
        current_joint_state.positions[i] + relative_desired_joint_state_vec_[i];
      desired_joint_state.velocities[i] = relative_desired_joint_state_vec_[i] / 0.1;//period.seconds();
      // TODO(destogl): for now acceleration commands are not used but here simply resetted
      desired_joint_state.accelerations[i] = 0.0;
      // TODO(destogl): in the future we need here to remember previously commanded velocity
      // desired_joint_state.accelerations[i] = desired_joint_state.velocities[i] / period.seconds();
    }
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("AdmittanceRule"),
      "Conversion of Cartesian deltas to joint deltas failed. Sending current joint values to the "
      "robot.");
    desired_joint_state.positions = current_joint_state.positions;
    std::fill(desired_joint_state.velocities.begin(), desired_joint_state.velocities.end(), 0.0);
    std::fill(desired_joint_state.accelerations.begin(), desired_joint_state.accelerations.end(), 0.0);
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_RULE_IMPL_HPP_
