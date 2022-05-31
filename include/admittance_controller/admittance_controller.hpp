// Copyright (c) 2022, PickNik, Inc.
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
/// \authors: Denis Stogl, Andy Zelenak, Paul Gesel

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "admittance_controller/admittance_rule.hpp"
#include "admittance_controller/visibility_control.h"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits_interface/joint_limits_interface.hpp"
#include "joint_limits_interface/joint_limits.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "semantic_components/force_torque_sensor.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "joint_trajectory_controller/trajectory_execution_impl.hpp"


// TODO(destogl): this is only temporary to work with servo. It should be either trajectory_msgs/msg/JointTrajectoryPoint or std_msgs/msg/Float64MultiArray
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "joint_trajectory_controller/trajectory_execution_impl.hpp"
#include "joint_trajectory_controller/tolerances.hpp"

using namespace std::chrono_literals;

namespace admittance_controller
{
    using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
    using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;

    struct RTBuffers{
        realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> input_traj_command;
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::WrenchStamped>> input_wrench_command_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> input_pose_command_;
        std::shared_ptr<RealtimeGoalHandle> rt_active_goal_;
        std::unique_ptr<realtime_tools::RealtimePublisher<ControllerStateMsg>> state_publisher_;
    };

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface,
                             public joint_trajectory_controller::TrajectoryExecutionImpl
{
public:
    ADMITTANCE_CONTROLLER_PUBLIC
    AdmittanceController();

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_init() override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    ADMITTANCE_CONTROLLER_PUBLIC
    controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;


protected:
    std::vector<std::string> joint_names_;
    int num_joints_{};
    std::vector<std::string> command_interface_types_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_acceleration_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
    std::vector<std::string> state_interface_types_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_acceleration_state_interface_;
    // Admittance rule and dependent variables;
    std::unique_ptr<admittance_controller::AdmittanceRule> admittance_;
    // joint limiter
    using JointLimiter = joint_limits_interface::PositionJointSaturationHandle;
    std::shared_ptr<pluginlib::ClassLoader<JointLimiter>> joint_limiter_loader_;
    std::unique_ptr<JointLimiter> joint_limiter_;
    std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;
    // controller parameters filled by ROS
    std::string ft_sensor_name_;
    bool use_joint_commands_as_input_{};
    std::string joint_limiter_type_;
    bool allow_partial_joints_goal_{};
    bool allow_integration_in_goal_trajectories_{};
    double action_monitor_rate{};
    bool open_loop_control_;
    // ROS subscribers
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr input_joint_command_subscriber_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr input_wrench_command_subscriber_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_pose_command_subscriber_ = nullptr;
    rclcpp::Publisher<control_msgs::msg::AdmittanceControllerState>::SharedPtr  s_publisher_ = nullptr;
    // ROS messages
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_command_msg;
    std::shared_ptr<geometry_msgs::msg::WrenchStamped> wrench_msg;
    std::shared_ptr<geometry_msgs::msg::PoseStamped> pose_command_msg;
    // ROS Transformation lookup variables
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // real-time buffers
    RTBuffers rtBuffers;
    joint_trajectory_controller::SegmentTolerances default_tolerances_;
    // controller running state
    bool controller_is_active_{};
    const std::vector<std::string> allowed_state_interface_types_ = {
            hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY,
    };
    const std::vector<std::string> allowed_command_interface_types_ = {
                hardware_interface::HW_IF_POSITION,
                hardware_interface::HW_IF_VELOCITY,
                };
    // last time update or on activate was run
    rclcpp::Time last_state_publish_time_;
    trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
    trajectory_msgs::msg::JointTrajectoryPoint last_state_reference_;
    trajectory_msgs::msg::JointTrajectoryPoint prev_trajectory_point_;
    // control loop data
    trajectory_msgs::msg::JointTrajectoryPoint state_reference, state_current, state_desired,
            state_error;
    trajectory_msgs::msg::JointTrajectory pre_admittance_point;
    // held references
    rclcpp::TimerBase::SharedPtr goal_handle_timer_;
    // helper methods
    void joint_trajectory_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);
    void wrench_stamped_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg);
    void pose_stamped_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
    void read_state_from_hardware(trajectory_msgs::msg::JointTrajectoryPoint & state);
    void read_state_from_command_interfaces(trajectory_msgs::msg::JointTrajectoryPoint & state);
    bool get_string_array_param_and_error_if_empty(std::vector<std::string> & parameter, const char * parameter_name);
    bool get_string_param_and_error_if_empty(std::string & parameter, const char * parameter_name);
    bool get_bool_param_and_error_if_empty (bool & parameter, const char * parameter_name);
    bool get_double_param_and_error_if_empty (double & parameter, const char * parameter_name);

};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
