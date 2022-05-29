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

#include "admittance_controller/admittance_controller.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>

#include "admittance_controller/admittance_rule_impl.hpp"
#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits_interface/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface()
{
}

CallbackReturn AdmittanceController::on_init()
{
  admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();

  admittance_->parameters_.initialize(get_node());



  try {
//      raise(SIGTRAP);
    // TODO: use variables as parameters
//    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>({}));
//    get_node()->declare_parameter<std::vector<std::string>>("command_interfaces", std::vector<std::string>({}));
//    get_node()->declare_parameter<std::vector<std::string>>("state_interfaces", std::vector<std::string>({}));
//    get_node()->declare_parameter<std::string>("ft_sensor_name", "");
//    get_node()->declare_parameter<std::string>("joint_limiter_type", "joint_limits/SimpleJointLimiter");
//    get_node()->declare_parameter<bool>("use_joint_commands_as_input", false);
//    get_node()->declare_parameter<double>("action_monitor_rate", 20.0);
//    get_node()->declare_parameter<bool>("allow_partial_joints_goal", allow_partial_joints_goal_);
//    get_node()->declare_parameter<bool>(
//      "allow_integration_in_goal_trajectories", allow_integration_in_goal_trajectories_);
//    get_node()->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
//    get_node()->declare_parameter<double>("constraints.goal_time", 0.0);

    // TODO(destogl): enable when IK-plugin support is added
    // get_node()->declare_parameter<std::string>("IK.plugin", "");
    admittance_->parameters_.declare_parameters();

    auto jointnames = get_node()->get_parameter("joints").as_string_array();

      RCLCPP_INFO(get_node()->get_logger(), jointnames[0].c_str());

//      using namespace std::chrono_literals;
//      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(get_node());
//      while (!parameters_client->wait_for_service(1s)) {
//          if (!rclcpp::ok()) {
//              RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
//              rclcpp::shutdown();
//          }
//          RCLCPP_INFO(get_node()->get_logger(), "service not available, waiting again...");
//      }
//
//      allow_integration_in_goal_trajectories_ = true;
//
//      // Set several different types of parameters.
//      auto set_parameters_results = parameters_client->set_parameters(
//              {
//                  rclcpp::Parameter("foo", 2),
//                  rclcpp::Parameter("joints", std::vector<std::string>({"shoulder_pan_joint",
//                                                                        "shoulder_lift_joint",
//                                                                        "elbow_joint",
//                                                                        "wrist_1_joint",
//                                                                        "wrist_2_joint",
//                                                                        "wrist_3_joint"})),
//              rclcpp::Parameter("command_interfaces", std::vector<std::string>({"position"})),
//              rclcpp::Parameter("state_interfaces", std::vector<std::string>({"position", "velocity"})),
//              rclcpp::Parameter("ft_sensor_name", "tcp_fts_sensor"),
//              rclcpp::Parameter("joint_limiter_type", "joint_limits/SimpleJointLimiter"),
//              rclcpp::Parameter("use_joint_commands_as_input", true),
//              rclcpp::Parameter("action_monitor_rate", 20.0),
//              rclcpp::Parameter("allow_partial_joints_goal", allow_partial_joints_goal_),
//              rclcpp::Parameter("allow_integration_in_goal_trajectories", allow_integration_in_goal_trajectories_),
//              rclcpp::Parameter("constraints.stopped_velocity_tolerance", 0.01),
//              rclcpp::Parameter("constraints.goal_time", 0.1),
//              });
//      // Check to see if they were set.
//      for (auto & result : set_parameters_results) {
//          if (!result.successful) {
//              RCLCPP_ERROR(get_node()->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
//          }
//      }


  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty = [&](
    std::vector<std::string> & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string_array();
    return error_if_empty(parameter, parameter_name);
  };

  auto get_string_param_and_error_if_empty = [&](
    std::string & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  // TODO(destogl): If we would use C++20 than we can use templates here
  auto get_bool_param_and_error_if_empty = [&](
    bool & parameter, const char * parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).get_value<bool>();
    return false; // TODO(destogl): how to check "if_empty" for bool?
  };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
    get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
    get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
    get_bool_param_and_error_if_empty(use_joint_commands_as_input_, "use_joint_commands_as_input") ||
    get_string_param_and_error_if_empty(joint_limiter_type_, "joint_limiter_type") ||

    !admittance_->parameters_.get_parameters()
    )
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error happened during reading parameters");
    return CallbackReturn::ERROR;
  }

  try {
    admittance_->filter_chain_ =
    std::make_unique<filters::FilterChain<geometry_msgs::msg::WrenchStamped>>(
      "geometry_msgs::msg::WrenchStamped");
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during filter chain creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

//  if (!admittance_->filter_chain_->configure("input_wrench_filter_chain",
//    get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
//  {
//    RCLCPP_ERROR(get_node()->get_logger(),
//                 "Could not configure sensor filter chain, please check if the "
//                 "parameters are provided correctly.");
//    return CallbackReturn::ERROR;
//  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Command interface type '" << interface << "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_POSITION)) {
    has_position_command_interface_ = true;
  }
  if (controller_interface::interface_list_contains_interface_type(
    command_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_command_interface_ = true;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_interface_types_.size());
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    if (it == allowed_interface_types_.end()) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << interface << "' not allowed!");
      return CallbackReturn::ERROR;
    }
  }

  if (!controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_POSITION)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "State interface type '" << std::string(hardware_interface::HW_IF_POSITION) << "' has to be always present allowed!");
    return CallbackReturn::ERROR;
  }

  if (controller_interface::interface_list_contains_interface_type(
    state_interface_types_, hardware_interface::HW_IF_VELOCITY)) {
    has_velocity_state_interface_ = true;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types) {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index) {
      if (index != 0) {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
    get_interface_list(command_interface_types_).c_str(),
    get_interface_list(state_interface_types_).c_str());

  auto num_joints = joint_names_.size();

  // Initialize joint limits
//  if (!joint_limiter_type_.empty())
//  {
//    RCLCPP_INFO(
//      get_node()->get_logger(), "Using joint limiter plugin: '%s'", joint_limiter_type_.c_str());
//    joint_limiter_loader_ = std::make_shared<pluginlib::ClassLoader<JointLimiter>>(
//      "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointLimits>");
//    joint_limiter_ = std::unique_ptr<JointLimiter>(
//      joint_limiter_loader_->createUnmanagedInstance(joint_limiter_type_));
////    joint_limiter_->init(joint_names_, get_node());
//  }
//  else
//  {
//    RCLCPP_INFO(get_node()->get_logger(), "Not using joint limiter plugin as none defined.");
//  }

  // subscriber callback
  // non realtime
  // TODO(karsten): check if traj msg and point time are valid
  auto callback = [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) -> void {
    if (!validate_trajectory_msg(
          *msg, allow_partial_joints_goal_, joint_names_, allow_integration_in_goal_trajectories_,
          get_node()->now()))
    {
      return;
    }

    // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
    // always replace old msg with new one for now
    if (subscriber_is_active_)
    {
      add_new_trajectory_msg(msg);
    }
  };

  // TODO(karsten1987): create subscriber with subscription deactivated
  joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(), callback);

  // Initialize FTS semantic semantic_component
  force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
    semantic_components::ForceTorqueSensor(ft_sensor_name_));

  // TODO(destogl): Add subscriber for velocity scaling

  // State publisher
  s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);


  // Initialize state message
  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.actual_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.desired_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->msg_.error_joint_state.positions.resize(num_joints, 0.0);
  state_publisher_->unlock();

  last_commanded_state_.positions.resize(num_joints);
  last_commanded_state_.velocities.resize(num_joints, 0.0);
  last_commanded_state_.accelerations.resize(num_joints, 0.0);

  if (use_joint_commands_as_input_) {
    RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Admittance controller does not support non-joint input modes.");
    return CallbackReturn::ERROR;
  }

  // action server configuration
  allow_partial_joints_goal_ = get_node()->get_parameter("allow_partial_joints_goal").get_value<bool>();
  if (allow_partial_joints_goal_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Goals with partial set of joints are allowed");
  }

  // Configure AdmittanceRule
  admittance_->configure(get_node());

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {

      return admittance_->parameters_.set_parameter_callback(parameters);
    });

  // HACK: This is workaround because it seems that updating parameters only in `on_activate` does
  // not work properly
  admittance_->parameters_.update();

  default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(*get_node(), joint_names_);

  // Read parameters customizing controller for special cases
  allow_integration_in_goal_trajectories_ =
          get_node()->get_parameter("allow_integration_in_goal_trajectories").get_value<bool>();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : joint_names_) {
    for (const auto & interface : command_interface_types_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(6 + joint_names_.size() * state_interface_types_.size());

  state_interfaces_config.names = force_torque_sensor_->get_state_interface_names();

  for (const auto & joint : joint_names_) {
    for (const auto & interface : state_interface_types_) {
      state_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return state_interfaces_config;
}

CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  const auto num_joints = joint_names_.size();

  // Update dynamic parameters before controller is started
  admittance_->parameters_.update();

  // order all joints in the storage
  for (const auto & interface : command_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
              get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_command_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : state_interface_types_) {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
              get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                   num_joints, interface.c_str(), joint_state_interface_[index].size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(joint_state_interface_[0].size());
  for (size_t index = 0; index < joint_state_interface_[0].size(); ++index)
  {
    traj_msg_home_ptr_->points[0].positions[index] =
      joint_state_interface_[0][index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_home_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;

  // Initialize current state storage if hardware state has tracking offset
  resize_joint_trajectory_point(
    last_commanded_state_, joint_names_.size(), has_velocity_state_interface_,
    has_acceleration_state_interface_);
  read_state_from_hardware(last_commanded_state_);

  // Initialize interface of the FTS semantic semantic component
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize Admittance Rule from current states
  admittance_->reset();

  read_state_from_hardware(last_commanded_state_);
  if (joint_limiter_)
  {
//    joint_limiter_->configure(last_commanded_state_);
  }
  // Handle restart of controller by reading last_commanded_state_ from commands if not nan
  read_state_from_command_interfaces(last_commanded_state_);

  last_state_reference_ = last_commanded_state_;
//   prev_trajectory_point_ = last_commanded_state_;

  // Set initial command values - initialize all to simplify update
  std::shared_ptr<ControllerCommandWrenchMsg> msg_wrench = std::make_shared<ControllerCommandWrenchMsg>();
  msg_wrench->header.frame_id = admittance_->parameters_.control_frame_;
  input_wrench_command_.writeFromNonRT(msg_wrench);

  std::shared_ptr<ControllerCommandJointMsg> msg_joint = std::make_shared<ControllerCommandJointMsg>();
  msg_joint->joint_names = joint_names_;
  msg_joint->points.reserve(1);

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
  trajectory_point.positions.reserve(num_joints);
  trajectory_point.velocities.resize(num_joints, 0.0);
  // FIXME(destogl): ATTENTION: This does not work properly, so using velocity mode and commenting positions out!
//   for (auto index = 0u; index < num_joints; ++index) {
//     trajectory_point.positions.emplace_back(joint_state_interface_[0][index].get().get_value());
//   }
  msg_joint->points.emplace_back(trajectory_point);

  input_joint_command_.writeFromNonRT(msg_joint);

  // TODO(destogl): Move this to the Cartesian admittance controller
  std::shared_ptr<ControllerCommandPoseMsg> msg_pose = std::make_shared<ControllerCommandPoseMsg>();
  msg_pose->header.frame_id = admittance_->parameters_.control_frame_;
  if (admittance_->get_pose_of_control_frame_in_base_frame(*msg_pose) !=
      controller_interface::return_type::OK)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Can not find transform from '%s' to '%s' needed in the update loop",
                 admittance_->parameters_.ik_base_frame_.c_str(), admittance_->parameters_.control_frame_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  input_pose_command_.writeFromNonRT(msg_pose);

  const double action_monitor_rate =
          get_node()->get_parameter("action_monitor_rate").get_value<double>();

  create_action_server(
          get_node(), this, action_monitor_rate, allow_partial_joints_goal_, joint_names_,
    allow_integration_in_goal_trajectories_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the robot
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    joint_command_interface_[0][index].get().set_value(
      joint_command_interface_[0][index].get().get_value());
  }

  for (auto index = 0ul; index < allowed_interface_types_.size(); ++index) {
    joint_command_interface_[index].clear();
    joint_state_interface_[index].clear();
  }
  release_interfaces();

  release_action_server();

  force_torque_sensor_->release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &)
{
  // go home
  traj_home_point_ptr_->update(traj_msg_home_ptr_);
  traj_point_active_ptr_ = &traj_home_point_ptr_;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (admittance_->parameters_.enable_parameter_update_without_reactivation_)
  {
    admittance_->parameters_.update();
  }

  // Check if a new external message has been received from nonRT threads
  check_for_new_trajectory(joint_names_, joint_command_interface_);

  trajectory_msgs::msg::JointTrajectoryPoint state_reference, state_current, state_desired,
    state_error;
  trajectory_msgs::msg::JointTrajectory pre_admittance_point;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(
    state_reference, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);
  resize_joint_trajectory_point(
    state_current, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);
  resize_joint_trajectory_point(
    state_desired, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);
  resize_joint_trajectory_point(
    state_error, joint_num, has_velocity_state_interface_, has_acceleration_state_interface_);

  // current state update
  state_current.time_from_start.set__sec(0);
  read_state_from_hardware(state_current);

  if (admittance_->parameters_.open_loop_control_) {
    // TODO(destogl): This may not work in every case.
    // Please add checking which states are available and which not!
    state_current = last_commanded_state_;
  }

  // admittance rule
  geometry_msgs::msg::Wrench ft_values;
  force_torque_sensor_->get_values_as_message(ft_values);

  const size_t num_joints = joint_names_.size();

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&, joint_num](auto & joint_inteface, const std::vector<double> & trajectory_point_interface) {
      for (auto index = 0ul; index < joint_num; ++index)
      {
        joint_inteface[index].get().set_value(trajectory_point_interface[index]);
      }
    };

  bool valid_trajectory_point = false;
  // Used only when we have a valid trajectory
  bool before_last_point = false;
  bool abort = false;
  bool outside_goal_state_tolerance = false;
  joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;

  // currently carrying out a trajectory
  if (have_trajectory())
  {
    // if we will be sampling for the first time, prefix the trajectory with the current state
    // TODO(destogl): Should we use `time` argument?
    set_point_before_trajectory_msg(
      open_loop_control_, get_node()->now(), state_current, last_state_reference_);

    // find segment for current timestamp
    // joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;

    valid_trajectory_point =
      sample_trajectory(get_node()->now(), state_reference, start_segment_itr, end_segment_itr);
    if (valid_trajectory_point)
    {
      last_state_reference_ = state_reference;
      last_state_reference_.velocities.assign(num_joints, 0.0);
      last_state_reference_.accelerations.assign(num_joints, 0.0);
    }

    before_last_point = is_before_last_point(end_segment_itr);
//     prev_trajectory_point_ = state_desired;
  }

  // If there is no servo or trajectory command -> keep the same reference
  if (!valid_trajectory_point)
  {
    state_reference = last_state_reference_;
  }

  pre_admittance_point.points.push_back(state_reference);

  if (state_reference.positions.empty())
  {
    state_desired.positions.assign(num_joints, 0.0);
  }
  else
  {
    state_desired.positions = state_reference.positions;
  }
  state_desired.velocities.assign(num_joints, 0.0);
  state_desired.accelerations.assign(num_joints, 0.0);

  admittance_->update(state_current, ft_values, state_reference, period, state_desired);

  // Apply joint limiter
  if (joint_limiter_)
  {
    joint_limiter_->enforce_limits(period);
  }

  // Compute state_error
  auto compute_error_for_joint = [&](
                                   trajectory_msgs::msg::JointTrajectoryPoint & error, int index,
                                   const trajectory_msgs::msg::JointTrajectoryPoint & current,
                                   const trajectory_msgs::msg::JointTrajectoryPoint & desired) {
    // error defined as the difference between current and desired
    error.positions[index] =
      angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    if (has_velocity_state_interface_ && has_velocity_command_interface_)
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  for (auto index = 0ul; index < joint_num; ++index)
  {
    compute_error_for_joint(state_error, index, state_current, state_desired);

    // Check if trajectory is complete or should be aborted
    if(valid_trajectory_point) {
      if (
        before_last_point &&
        !check_state_tolerance_per_joint(
          state_error, index, default_tolerances_.state_tolerance[index], false))
      {
        abort = true;
      }
      // past the final point, check that we end up inside goal tolerance
      if (
        !before_last_point &&
        !check_state_tolerance_per_joint(
          state_error, index, default_tolerances_.goal_state_tolerance[index], false))
      {
        outside_goal_state_tolerance = true;
      }
    }
  }

  // Write new joint angles to the robot
  // set values for next hardware write()
  if (has_position_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[0], state_desired.positions);
  }
  if (has_velocity_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[1], state_desired.velocities);
  }
  if (has_acceleration_command_interface_)
  {
    assign_interface_from_point(joint_command_interface_[2], state_desired.accelerations);
  }
  // TODO(anyone): Add here "if using_closed_loop_hw_interface_adapter" (see ROS1) - #171
  //       if (check_if_interface_type_exist(
  //           command_interface_types_, hardware_interface::HW_IF_EFFORT)) {
  //         assign_interface_from_point(joint_command_interface_[3], state_desired.effort);
  //       }

  if(valid_trajectory_point) {
    // handle action server feedback
    perform_action_server_update(
      before_last_point, abort, outside_goal_state_tolerance,
      default_tolerances_.goal_time_tolerance, get_node()->now(), joint_names_, state_current,
      state_desired, state_error, start_segment_itr);
  }

  // store command as state when hardware state has tracking offset
  // TODO: Should we be checking if open_loop_control_ around this assignment?
  last_commanded_state_ = state_desired;

  // Publish controller state
  state_publisher_->lock();
  state_publisher_->msg_.input_joint_command = pre_admittance_point;
  state_publisher_->msg_.desired_joint_state = state_desired;
  state_publisher_->msg_.actual_joint_state = state_current;
  state_publisher_->msg_.error_joint_state = state_error;
  admittance_->get_controller_state(state_publisher_->msg_);
  state_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

void AdmittanceController::read_state_from_hardware(
  trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  const auto num_joints = joint_names_.size();
  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (auto index = 0ul; index < num_joints; ++index) {
      trajectory_point_interface[index] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, joint_state_interface_[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    assign_point_from_interface(state.velocities, joint_state_interface_[1]);
    // Acceleration is used only in combination with velocity
    // TODO(destogl): enable acceleration and remove next line
    state.accelerations.clear();
//     if (has_acceleration_state_interface_) {
//       assign_point_from_interface(state.accelerations, joint_state_interface_[2]);
//     } else {
//       // Make empty so the property is ignored during interpolation
//       state.accelerations.clear();
//     }
  } else {
    // Make empty so the property is ignored during interpolation
    state.velocities.clear();
    state.accelerations.clear();
  }
}

bool AdmittanceController::read_state_from_command_interfaces(
  trajectory_msgs::msg::JointTrajectoryPoint & output_state)
{
  bool has_values = true;
  const auto num_joints = joint_names_.size();
  trajectory_msgs::msg::JointTrajectoryPoint state = output_state;

  auto assign_point_from_interface = [&, num_joints](
    std::vector<double> & trajectory_point_interface, const auto & joint_interface)
    {
      for (auto index = 0ul; index < num_joints; ++index) {
        trajectory_point_interface[index] = joint_interface[index].get().get_value();
      }
    };

  auto interface_has_values = [](const auto & joint_interface)
    {
      return std::find_if(
        joint_interface.begin(), joint_interface.end(),
        [](const auto & interface) {return std::isnan(interface.get().get_value());}) ==
             joint_interface.end();
    };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(joint_command_interface_[0])) {
    assign_point_from_interface(state.positions, joint_command_interface_[0]);
  } else {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_) {
    if (has_velocity_command_interface_ && interface_has_values(joint_command_interface_[1])) {
      assign_point_from_interface(state.velocities, joint_command_interface_[1]);
      //TODO(destogl): enable this line under to be sure if positions are not existing and velocities
      // are existing to still update the output_state; !commented because not tested!
//       has_values = true;
    } else {
      state.velocities.clear();
      has_values = false;
    }
  }
  else {
    state.velocities.clear();
  }

// TODO(destogl): Enable this
//   // Acceleration is used only in combination with velocity
//   if (has_acceleration_state_interface_) {
//     if (has_acceleration_command_interface_ && interface_has_values(joint_command_interface_[2])) {
//       assign_point_from_interface(state.accelerations, joint_command_interface_[2]);
//     } else {
//       state.accelerations.clear();
//       has_values = false;
//     }
//   } else {
//     state.accelerations.clear();
//   }

  if (has_values) {
    output_state = state;
  }

  return has_values;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController, controller_interface::ControllerInterface)
