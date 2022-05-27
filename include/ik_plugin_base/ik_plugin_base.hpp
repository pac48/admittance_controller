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
/// \author: Andy Zelenak
/// \description: Base class for differential kinematics plugins

#ifndef IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_
#define IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "rclcpp/rclcpp.hpp"


namespace ik_plugin_base
{
class IKBaseClass
{
public:
  IKBaseClass() = default;
  virtual ~IKBaseClass() = default;

  /**
   * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
   * It uses the Jacobian from MoveIt.
   */
  virtual bool
  initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & group_name) = 0;

  /**
   * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
   * \param delta_x_vec input Cartesian deltas (x, y, z, rx, ry, rz)
   * \param control_frame_to_ik_base transform the requested delta_x to ik_base frame
   * \param delta_theta_vec output vector with joint states
   * \return true if successful
   */
  virtual bool
  convert_cartesian_deltas_to_joint_deltas(
    std::vector<double> & delta_x_vec,
    const geometry_msgs::msg::TransformStamped & control_frame_to_ik_base,
    std::vector<double> & delta_theta_vec) = 0;

  /**
   * \brief Convert joint delta-theta to Cartesian delta-x, using the Jacobian.
   * \param[in] delta_theta_vec vector with joint states
   * \param[in] tf_ik_base_to_desired_cartesian_frame transformation to the desired Cartesian frame. Use identity matrix to stay in the ik_base frame.
   * \param[out] delta_x_vec  Cartesian deltas (x, y, z, rx, ry, rz)
   * \return true if successful
   */
  virtual bool
  convert_joint_deltas_to_cartesian_deltas(
    std::vector<double> &  delta_theta_vec,
    const geometry_msgs::msg::TransformStamped & tf_ik_base_to_desired_cartesian_frame,
    std::vector<double> & delta_x_vec) = 0;

  /**
   * \brief Update the state of the robot in the IK solver
   */
  virtual bool update_robot_state(const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state) = 0;
};

}  // namespace ik_plugin_base

#endif  // IK_PLUGIN_BASE__IK_PLUGIN_BASE_HPP_
