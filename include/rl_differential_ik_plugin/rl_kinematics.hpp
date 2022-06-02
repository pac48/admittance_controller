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
/// \author: Paul Gesel

#pragma once

#include "eigen3/Eigen/Core"

#include "ik_interface/ik_plugin_base.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/math/Matrix.h"
#include "rl/math/Unit.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Model.h"
#include "rl/mdl/Body.h"

#include "tf2_eigen/tf2_eigen.hpp"


namespace rl_differential_ik_plugin
{

class RLKinematics : public ik_interface::IKBaseClass
{
public:
  RLKinematics();

  /**
   * \brief Create an object which takes Cartesian delta-x and converts to joint delta-theta.
   * It uses the Jacobian from MoveIt.
   */
  bool initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & group_name);

  /**
   * \brief Convert Cartesian delta-x to joint delta-theta, using the Jacobian.
   * \param delta_x_vec input Cartesian deltas (x, y, z, rx, ry, rz)
   * \param control_frame_to_ik_base transform the requested delta_x to MoveIt's ik_base frame
   * \param delta_theta_vec output vector with joint states
   * \return true if successful
   */
  bool
  convert_cartesian_deltas_to_joint_deltas(
    std::vector<double> & delta_x_vec,
    const geometry_msgs::msg::TransformStamped & control_frame_to_ik_base,
    std::vector<double> & delta_theta_vec);

  /**
   * \brief Convert joint delta-theta to Cartesian delta-x, using the Jacobian.
   * \param[in] delta_theta_vec vector with joint states
   * \param[in] tf_ik_base_to_desired_cartesian_frame transformation to the desired Cartesian frame. Use identity matrix to stay in the ik_base frame.
   * \param[out] delta_x_vec  Cartesian deltas (x, y, z, rx, ry, rz)
   * \return true if successful
   */
  bool
  convert_joint_deltas_to_cartesian_deltas(
    std::vector<double> &  delta_theta_vec,
    const geometry_msgs::msg::TransformStamped & tf_ik_base_to_desired_cartesian_frame,
    std::vector<double> & delta_x_vec);

  bool update_robot_state(const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_state)
  {
    if (current_joint_state.positions.size() != model.getPosition().size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Vector size mismatch in update_robot_state()");
      return false;
    }

    auto positions = model.getPosition();
    for(int i =0; i < positions.size(); i++){
        positions[i] = current_joint_state.positions[i];
    }
    model.setPosition(positions);
    model.forwardPosition();

    return true;
  }

private:
  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
//   */

  void calculateJacobian();
//  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
//                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
//                                             const Eigen::MatrixXd& pseudo_inverse);

//    Eigen::Isometry3d get_link_transform(
//            const std::string& link_name, const trajectory_msgs::msg::JointTrajectoryPoint & joint_state);
//

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
//    rl::mdl::Model model;
    rl::mdl::Dynamic model;

  // Pre-allocate for speed
  Eigen::MatrixXd all_jacobians_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd matrix_s_;
  Eigen::MatrixXd pseudo_inverse_;

        int numEE;
        int numDof;
        int offseti;
        int offsetj;

};

}  // namespace moveit_differential_ik_plugin
