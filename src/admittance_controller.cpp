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

#include "admittance_controller/admittance_controller.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include "admittance_controller/admittance_rule_impl.hpp"
#include "filters/filter_chain.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "joint_limits_interface/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace admittance_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


    AdmittanceController::AdmittanceController() = default;

    void AdmittanceController::joint_trajectory_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg){
        if (!validate_trajectory_msg(
                *msg, allow_partial_joints_goal_, joint_names_,
                allow_integration_in_goal_trajectories_,get_node()->now())){
            return;
        }
//        // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
//        // always replace old msg with new one for now
        if (controller_is_active_)
        {
            rtBuffers.input_traj_command.writeFromNonRT(msg);
        }
    }
    void AdmittanceController::wrench_stamped_callback(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> msg){
        if (controller_is_active_)
        {
            rtBuffers.input_wrench_command_.writeFromNonRT(msg);
        }
    }
    void AdmittanceController::pose_stamped_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg){
        if (controller_is_active_)
        {
            rtBuffers.input_pose_command_.writeFromNonRT(msg);
        }
    }

    CallbackReturn AdmittanceController::on_init() {
        // load controller parameters
        admittance_ = std::make_unique<admittance_controller::AdmittanceRule>();
        admittance_->parameters_.initialize(get_node());

        if (get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
            get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
            get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces"))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error happened during reading parameters");
            return CallbackReturn::ERROR;
        }

        for (auto tmp : state_interface_types_){
            RCLCPP_INFO(get_node()->get_logger(), ("state int types are: "+ tmp+"\n").c_str());
        }
        for (auto tmp : command_interface_types_){
            RCLCPP_INFO(get_node()->get_logger(), ("command int types are: "+ tmp+"\n").c_str());

        }

        try{
            admittance_->parameters_.declare_parameters();
        } catch (const std::exception & e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        // initialize RTbuffers
        rtBuffers.input_pose_command_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::PoseStamped>());
        rtBuffers.input_wrench_command_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::WrenchStamped>());
        rtBuffers.input_pose_command_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::PoseStamped>());
        rtBuffers.input_traj_command.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration AdmittanceController::command_interface_configuration() const {
        // Create an InterfaceConfiguration for the controller manager. The controller manager will then try to
        // claim the joint + interface combinations for command interfaces from the resource manager. Finally,
        // controller manager will populate the command_interfaces_ vector field via the ControllerInterfaceBase
        // Note: command_interface_types_ contains position, velocity; acceleration, effort are not supported

        std::vector<std::string> command_interfaces_config_names;//(joint_names_.size() * command_interface_types_.size());

        for (const auto & interface : command_interface_types_) {
            for (const auto & joint : joint_names_) {
                command_interfaces_config_names.push_back(joint + "/" + interface);
            }
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL,
                command_interfaces_config_names};
    }

    controller_interface::InterfaceConfiguration AdmittanceController::state_interface_configuration() const {
        // Create an InterfaceConfiguration for the controller manager. The controller manager will then try to
        // claim the joint + interface combinations for state interfaces from the resource manager. Finally,
        // controller manager will populate the state_interfaces_ vector field via the ControllerInterfaceBase.
        // Note: state_interface_types_ contains position, velocity, acceleration; effort is not supported

        std::vector<std::string> state_interfaces_config_names; //= force_torque_sensor_->get_state_interface_names();

        for (const auto & interface : state_interface_types_) {
            for (const auto & joint : joint_names_) {
                state_interfaces_config_names.push_back(joint + "/" + interface);
            }
        }
        auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
        state_interfaces_config_names.insert(state_interfaces_config_names.end(), ft_interfaces.begin(), ft_interfaces.end() );

        return {controller_interface::interface_configuration_type::INDIVIDUAL,
                state_interfaces_config_names};
    }

    template<typename T>
    bool check_and_assign_new_message(realtime_tools::RealtimeBuffer<std::shared_ptr<T>> buffer,
                                      std::shared_ptr<T>& current_external_msg){
        std::shared_ptr<T> new_external_msg = *buffer.readFromRT();
        if (current_external_msg == new_external_msg){
            return false;
        }
        current_external_msg = new_external_msg;
        return true;
    }

    controller_interface::return_type
    AdmittanceController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        // Realtime constraints are required in this function

//        RCLCPP_INFO(get_node()->get_logger(), "clock type: %d",time.get_clock_type());

        // check controller state
        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            return controller_interface::return_type::OK;
        }

        // sense: get all controller inputs
        auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
        traj_command_msg = *rtBuffers.input_traj_command.readFromRT();
        if (current_external_msg != traj_command_msg) {
            // this is a hack
            std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>> tmp = {
                    joint_position_command_interface_};
            fill_partial_goal(traj_command_msg, joint_names_, tmp);
            sort_to_local_joint_order(traj_command_msg, joint_names_);
            traj_external_point_ptr_->update(traj_command_msg);
        }
        if (check_and_assign_new_message(rtBuffers.input_pose_command_, pose_command_msg)) {
        }
        if (check_and_assign_new_message(rtBuffers.input_wrench_command_, wrench_msg)) {
        }

        geometry_msgs::msg::Wrench ft_values;
        force_torque_sensor_->get_values_as_message(ft_values);
        read_state_from_hardware(state_current);
        state_current.time_from_start.set__sec(0);

        // if values are not available, assume that the current state is the last commanded
        if (state_current.positions.empty()) {state_current.positions = last_commanded_state_.positions;}
        if (state_current.velocities.empty()) state_current.velocities = last_commanded_state_.velocities;
        if (state_current.accelerations.empty()) state_current.accelerations = last_commanded_state_.accelerations;

        // find segment for current timestamp. In the case that the trajectory sample is invalid, state_reference
        // will be set to empty.
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_itr, end_segment_itr;
        bool valid_trajectory_point = false;
        bool before_last_point = false;
        if (have_trajectory())
        { // do not sample trajectory if one does not exist
            valid_trajectory_point =
                    sample_trajectory(time, state_reference, start_segment_itr, end_segment_itr);
            before_last_point = is_before_last_point(end_segment_itr);
            // add starting offset to user command
            for (int i =0 ; i < state_offset_.positions.size(); i++){
                state_reference.positions[i] += state_offset_.positions[i];
            }
            last_state_reference_ = state_reference;
        }

        if (!valid_trajectory_point || !have_trajectory()){
            // if there is no current trajectory, then the reference should be the current state
            state_reference = last_state_reference_;
//            last_state_reference_ = state_current;
        }

        // save state reference before applying admittance rule
        pre_admittance_point.points[0] = state_reference;

        // command: determine desired state from trajectory or pose goal
        // and apply admittance controller

        admittance_->update(state_current, ft_values, state_reference, period, state_desired);

        RCLCPP_INFO(get_node()->get_logger(), "state_reference [%f, %f, %f]", state_reference.positions[0],state_reference.positions[1],state_reference.positions[2]);
        RCLCPP_INFO(get_node()->get_logger(), "state_desired [%f, %f, %f] ", state_desired.positions[0],state_desired.positions[1],state_desired.positions[2]);

        // Apply joint limiter
//        if (joint_limiter_) joint_limiter_->enforce_limits(period);

        // write calculated values to joint interfaces
        // at goal time (end of trajectory), check goal reference error and send fail to
        // action server out of tolerance
        for (auto i = 0ul; i < joint_position_command_interface_.size(); i++) {
            joint_position_command_interface_[i].get().set_value(state_desired.positions[i]);
            last_commanded_state_.positions[i] = state_desired.positions[i];
        }
        for (auto i = 0ul; i < joint_velocity_command_interface_.size(); i++) {
            joint_velocity_command_interface_[i].get().set_value(state_desired.velocities[i]);
            if (open_loop_control_ && joint_position_command_interface_.empty()){
                double dt = 1.0/60; // hack!
                last_commanded_state_.positions[i] += state_desired.velocities[i]*dt; // hack!
                joint_position_command_interface_[i].get().set_value(last_commanded_state_.positions[i]); // hack!
            }
            last_commanded_state_.velocities[i] = state_desired.velocities[i];
        }
        for (auto i = 0ul; i < joint_acceleration_command_interface_.size(); i++) {
            joint_acceleration_command_interface_[i].get().set_value(state_desired.accelerations[i]);
            last_commanded_state_.accelerations[i] = state_desired.accelerations[i];
        }
        // Compute state_error
        auto compute_error_for_joint = [&](
                trajectory_msgs::msg::JointTrajectoryPoint & error, int index,
                const trajectory_msgs::msg::JointTrajectoryPoint & current,
                const trajectory_msgs::msg::JointTrajectoryPoint & desired) {
            // error defined as the difference between current and desired
            error.positions[index] =
                    angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
            if (!joint_velocity_command_interface_.empty() && !joint_velocity_state_interface_.empty())
            {
                error.velocities[index] = desired.velocities[index] - current.velocities[index];
            }
            if (!joint_acceleration_command_interface_.empty() && !joint_acceleration_state_interface_.empty())
            {
                error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
            }
        };
        // abort if error violates tolerances
        bool abort = false;
        bool outside_goal_state_tolerance = false;
        for (auto index = 0ul; index < num_joints_; ++index){
            compute_error_for_joint(state_error, index, state_current, state_desired);
            abort |= before_last_point &&
                    !check_state_tolerance_per_joint(state_error, index, default_tolerances_.state_tolerance[index], false);
            outside_goal_state_tolerance |= !before_last_point && !check_state_tolerance_per_joint(
                                state_error, index, default_tolerances_.goal_state_tolerance[index], false);
        }
        perform_action_server_update(
                before_last_point, abort, outside_goal_state_tolerance,
                default_tolerances_.goal_time_tolerance, time, joint_names_, state_current,
                state_desired, state_error, start_segment_itr);

        // Publish controller state
        rtBuffers.state_publisher_->lock();
        rtBuffers.state_publisher_->msg_.input_joint_command = pre_admittance_point;
        rtBuffers.state_publisher_->msg_.desired_joint_state = state_desired;
        rtBuffers.state_publisher_->msg_.actual_joint_state = state_current;
        rtBuffers.state_publisher_->msg_.error_joint_state = state_error;
        admittance_->get_controller_state(rtBuffers.state_publisher_->msg_);
        rtBuffers.state_publisher_->unlockAndPublish();

        return controller_interface::return_type::OK;
    }

    CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        // load and set all ROS parameters
        if (get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
                get_string_array_param_and_error_if_empty(command_interface_types_, "command_interfaces") ||
                get_string_array_param_and_error_if_empty(state_interface_types_, "state_interfaces") ||
                get_string_param_and_error_if_empty(ft_sensor_name_, "ft_sensor_name") ||
                get_bool_param_and_error_if_empty(use_joint_commands_as_input_, "use_joint_commands_as_input") ||
                get_string_param_and_error_if_empty(joint_limiter_type_, "joint_limiter_type") ||
                get_bool_param_and_error_if_empty(allow_partial_joints_goal_, "allow_partial_joints_goal") ||
                get_bool_param_and_error_if_empty(allow_integration_in_goal_trajectories_, "allow_integration_in_goal_trajectories") ||
                get_double_param_and_error_if_empty(action_monitor_rate, "action_monitor_rate") ||
                get_bool_param_and_error_if_empty(open_loop_control_, "open_loop_control") ||
                !admittance_->parameters_.get_parameters()
                )
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error happened during reading parameters");
            return CallbackReturn::ERROR;
        }
        //  sort command_interface_types_
        auto command_interface = &allowed_command_interface_types_;
        std::sort(command_interface_types_.begin(), command_interface_types_.end(),
                  [command_interface](const std::string& lhs, const std::string& rhs)->bool {
                      auto it1 = std::find(command_interface->begin(),
                                           command_interface->end(), lhs);
                      auto it2 = std::find(command_interface->begin(),
                                           command_interface->end(), rhs);
                      return std::distance(command_interface->begin(), it1) < std::distance(command_interface->begin(), it2);
                  });
        // sort command_state_types_
        auto state_interface = &allowed_state_interface_types_;
        std::sort(state_interface_types_.begin(), state_interface_types_.end(),
                  [state_interface](const std::string& lhs, const std::string& rhs)->bool {
                      auto it1 = std::find(state_interface->begin(),
                                           state_interface->end(), lhs);
                      auto it2 = std::find(state_interface->begin(),
                                           state_interface->end(), rhs);
                      return std::distance(state_interface->begin(), it1) < std::distance(state_interface->begin(), it2);
                  });

        // Print output so users can be sure the interface setup is correct
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
        RCLCPP_INFO(
                get_node()->get_logger(), "Command interfaces are [%s] and and state interfaces are [%s].",
                get_interface_list(command_interface_types_).c_str(),
                get_interface_list(state_interface_types_).c_str());

        // action server configuration
        allow_partial_joints_goal_ = get_node()->get_parameter("allow_partial_joints_goal").get_value<bool>();
        if (allow_partial_joints_goal_){
            RCLCPP_INFO(get_node()->get_logger(), "Goals with partial set of joints are allowed");
        }
RCLCPP_INFO(get_node()->get_logger(), "Action status changes will be monitored at %.2f Hz.", action_monitor_rate);
        if (use_joint_commands_as_input_) {
            RCLCPP_INFO(get_node()->get_logger(), "Using Joint input mode.");
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "Admittance controller does not support non-joint input modes.");
            return CallbackReturn::ERROR;
        }

        // setup and start non-realtime threads, like subscribers and publishers
        input_joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
                std::bind(&AdmittanceController::joint_trajectory_callback, this, std::placeholders::_1));
        input_wrench_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
                "~/force_commands", rclcpp::SystemDefaultsQoS(),
                std::bind(&AdmittanceController::wrench_stamped_callback, this, std::placeholders::_1));
        input_pose_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
                "~/pose_commands", rclcpp::SystemDefaultsQoS(),
                std::bind(&AdmittanceController::pose_stamped_callback, this, std::placeholders::_1));
        // State publisher
        s_publisher_ = get_node()->create_publisher<control_msgs::msg::AdmittanceControllerState>(
                "~/state", rclcpp::SystemDefaultsQoS());
        rtBuffers.state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);
        // set up TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // assign state interfaces
        num_joints_ = joint_names_.size();
//        jointHandles.resize(num_joints_);

        // Initialize state message
        rtBuffers.state_publisher_->lock();
        rtBuffers.state_publisher_->msg_.joint_names = joint_names_;
        rtBuffers.state_publisher_->msg_.actual_joint_state.positions.resize(num_joints_, 0.0);
        rtBuffers.state_publisher_->msg_.desired_joint_state.positions.resize(num_joints_, 0.0);
        rtBuffers.state_publisher_->msg_.error_joint_state.positions.resize(num_joints_, 0.0);
        rtBuffers.state_publisher_->unlock();
        // get default tolerances
        default_tolerances_ = joint_trajectory_controller::get_segment_tolerances(*get_node(), joint_names_);
        // Initialize FTS semantic semantic_component
        force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(
                semantic_components::ForceTorqueSensor(ft_sensor_name_));

        // set up filter chain
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
        // TODO: this causes the error:
        //[ERROR] parameter 'input_wrench_filter_chain.filter1.name' has already been declared
//          if (!admittance_->filter_chain_->configure("input_wrench_filter_chain",
//            get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
//          {
//            RCLCPP_ERROR(get_node()->get_logger(),
//                         "Could not configure sensor filter chain, please check if the "
//                         "parameters are provided correctly.");
//            return CallbackReturn::ERROR;
//          }

        // TODO: this causes the error:
       //[ERROR]  error: package 'joint_limits' not found, searching:
        // Initialize joint limits

//

        // configure admittance rule
        admittance_->configure(get_node());
        // HACK: This is workaround because it seems that updating parameters only in `on_activate` does
        // not work properly: why?
        admittance_->parameters_.update();

        return LifecycleNodeInterface::on_configure(previous_state);
    }

    transmission_interface::JointHandle createJointHandle(const hardware_interface::LoanedCommandInterface & interface, double* val){
        return transmission_interface::JointHandle(interface.get_name(), interface.get_interface_name(), val);
    }
    transmission_interface::JointHandle createJointHandle(const hardware_interface::LoanedStateInterface& interface){
        return transmission_interface::JointHandle(interface.get_name(), interface.get_interface_name(), nullptr);
    } // interface.get_value())


    CallbackReturn AdmittanceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        // on_activate is called when the lifecycle activates. Realtime constraints are required.
        controller_is_active_ = true;
        // assign state interfaces
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>* joint_state_interfaces_ []
                = {&joint_position_state_interface_, &joint_velocity_state_interface_,
                   &joint_acceleration_state_interface_};
        for (auto i = 0ul; i < state_interface_types_.size(); i++) {
            for (auto j = 0ul; j < joint_names_.size(); j++) {
                (*joint_state_interfaces_[i]).emplace_back(state_interfaces_[i * num_joints_ + j]);
            }
        }
        // assign command interfaces
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>* joint_command_interfaces_ []
                = {&joint_position_command_interface_, &joint_velocity_command_interface_,
                   &joint_acceleration_command_interface_, &joint_effort_command_interface_};
        for (auto i = 0ul; i < command_interface_types_.size(); i++) {
            for (auto j = 0ul; j < joint_names_.size(); j++) {
                (*joint_command_interfaces_[i]).emplace_back(command_interfaces_[i * num_joints_ + j]);
            }
        }

        // create joint handle objects
//        for (int i; i < joint_position_command_interface_.size(); i++){
//            tmpHandles.push_back(createJointHandle(joint_position_state_interface_[i]));
//            tmpHandles.push_back(createJointHandle(joint_velocity_command_interface_[i],
//                                                   joint_velocity_command_interface_[i].get().get_value()));
//            joint_limits_interface::JointLimits jl;// = {0.0,0.0};
//            jointHandles[i] = JointLimiter(tmpHandles[tmpHandles.size()-2],tmpHandles[tmpHandles.size()-1], jl);
//
//        }


        // allocate memory for control loop data
        resize_joint_trajectory_point(last_commanded_state_, joint_names_.size(), !joint_velocity_command_interface_.empty(), !joint_acceleration_command_interface_.empty());
        resize_joint_trajectory_point(state_reference, joint_names_.size(), !joint_velocity_state_interface_.empty(), !joint_acceleration_state_interface_.empty());
        resize_joint_trajectory_point(state_current, joint_names_.size(), !joint_velocity_state_interface_.empty(), !joint_acceleration_state_interface_.empty());
        resize_joint_trajectory_point(state_desired, joint_names_.size(), !joint_velocity_state_interface_.empty(), !joint_acceleration_state_interface_.empty());
        resize_joint_trajectory_point(state_desired, joint_names_.size(), true, true);
        resize_joint_trajectory_point(state_error, joint_names_.size(), !joint_velocity_state_interface_.empty(), !joint_acceleration_state_interface_.empty());
        pre_admittance_point.points.push_back(last_commanded_state_);

        // Store 'home' pose
        traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        traj_msg_home_ptr_->header.stamp.sec = 0;
        traj_msg_home_ptr_->header.stamp.nanosec = 0;
        traj_msg_home_ptr_->points.resize(1);
        traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
        traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
        traj_msg_home_ptr_->points[0].positions.resize(joint_position_state_interface_.size());
        for (size_t index = 0; index < joint_position_state_interface_.size(); ++index)
        {
            traj_msg_home_ptr_->points[0].positions[index] = joint_position_state_interface_[index].get().get_value();
        }

        traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
        traj_home_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
        rtBuffers.input_traj_command.writeFromNonRT(
                std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

        subscriber_is_active_ = true;
        traj_point_active_ptr_ = &traj_external_point_ptr_;
        last_state_publish_time_ = get_node()->now();

        // Initialize interface of the FTS semantic semantic component
        force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
        // Initialize Admittance Rule from current states
        admittance_->reset();

        // Handle state after restart or initial startup
        read_state_from_hardware(last_state_reference_);
        read_state_from_command_interfaces(last_commanded_state_);
        // if last_state_reference_ is empty, we have no information about the state, assume zero
        if (last_state_reference_.positions.empty()) last_state_reference_.positions.assign(num_joints_, 0.0);
        if (last_state_reference_.velocities.empty()) last_state_reference_.velocities.assign(num_joints_, 0.0);
        if (last_state_reference_.accelerations.empty()) last_state_reference_.accelerations.assign(num_joints_, 0.0);
        // if in open loop mode, the position interface should be ignored even if it exist
        if (open_loop_control_){
            state_offset_ = last_state_reference_;
            joint_position_state_interface_.clear();
        }
        // if last_commanded_state_ is empty, then our safest option is to set it to the current state
        if (last_commanded_state_.positions.empty()) last_commanded_state_.positions = last_state_reference_.positions;
        if (last_commanded_state_.velocities.empty()) last_commanded_state_.velocities = last_state_reference_.velocities;
        if (last_commanded_state_.accelerations.empty()) last_commanded_state_.accelerations = last_state_reference_.accelerations;

        // if there are no state position interfaces, then force open loop control
        if (joint_position_state_interface_.empty() || !open_loop_control_){
            open_loop_control_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "control loop control set to true because no position state interface was provided. ");
        }

        create_action_server(
                get_node(), this, action_monitor_rate, allow_partial_joints_goal_, joint_names_,
                allow_integration_in_goal_trajectories_);

        return CallbackReturn::SUCCESS;;
    }

    CallbackReturn AdmittanceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        controller_is_active_ = false;
        force_torque_sensor_->release_interfaces();

        return LifecycleNodeInterface::on_deactivate(previous_state);
    }

    CallbackReturn AdmittanceController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        // go home
        traj_home_point_ptr_->update(traj_msg_home_ptr_);
        traj_point_active_ptr_ = &traj_home_point_ptr_;

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn AdmittanceController::on_error(const rclcpp_lifecycle::State &previous_state) {
        if (!reset())
        {
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    void AdmittanceController::read_state_from_hardware(
            trajectory_msgs::msg::JointTrajectoryPoint & state)
    {
        // Fill fields of state argument from hardware state interfaces. If the hardware does not exist,
        // the values are nan, that corresponding state field will be set to empty. If running in open loop
        // control, all states fields will be empty

        state.positions.resize(joint_position_state_interface_.size());
        state.velocities.resize(joint_velocity_state_interface_.size());
        state.accelerations.resize(joint_acceleration_state_interface_.size());

        // fill state message with values from hardware state interfaces
        for (auto i = 0ul; i < joint_position_state_interface_.size(); i++) {
            state.positions[i]  = joint_position_state_interface_[i].get().get_value();
            if (std::isnan(state.positions[i])){
                state.positions.clear();
                break;
            }
        }
        for (auto i = 0ul; i < joint_velocity_state_interface_.size(); i++) {
            state.velocities[i]  = joint_velocity_state_interface_[i].get().get_value();
            if (std::isnan(state.velocities[i])){
                state.velocities.clear();
                break;
            }
        }
        for (auto i = 0ul; i < joint_acceleration_state_interface_.size(); i++) {
            state.accelerations[i]  = joint_acceleration_state_interface_[i].get().get_value();
            if (std::isnan(state.accelerations[i])){
                state.positions.clear();
                break;
            }
        }
    }

    void AdmittanceController::read_state_from_command_interfaces(
            trajectory_msgs::msg::JointTrajectoryPoint & output_state)
    {
        // Fill fields of state argument from hardware command interfaces. If the interface does not exist or
        // the values are nan, that corresponding state field will be set to empty

        output_state.positions.resize(joint_position_command_interface_.size(), 0.0);
        output_state.velocities.resize(joint_velocity_command_interface_.size(), 0.0);
        output_state.accelerations.resize(joint_acceleration_command_interface_.size(), 0.0);

        // fill state message with values from hardware command interfaces
        for (auto i = 0ul; i < joint_position_command_interface_.size(); i++) {
            output_state.positions[i]  = joint_position_command_interface_[i].get().get_value();
            if (std::isnan(output_state.positions[i])){
                output_state.positions.clear();
                break;
            }
        }
        for (auto i = 0ul; i < joint_velocity_command_interface_.size(); i++) {
            output_state.velocities[i]  = joint_velocity_command_interface_[i].get().get_value();
            if (std::isnan(output_state.velocities[i])){
                output_state.velocities.clear();
                break;
            }
        }
        for (auto i = 0ul; i < joint_acceleration_command_interface_.size(); i++) {
            output_state.accelerations[i] = joint_acceleration_command_interface_[i].get().get_value();
            if (std::isnan(output_state.accelerations[i])){
                output_state.accelerations.clear();
                break;
            }
        }
    }

    bool AdmittanceController::get_string_array_param_and_error_if_empty(
            std::vector<std::string> & parameter, const char * parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).as_string_array();
        if (parameter.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
            return true;
        }
        return false;
    }

    bool AdmittanceController::get_string_param_and_error_if_empty(
            std::string & parameter, const char * parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).as_string();
        if (parameter.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
            return true;
        }
        return false;
    }

    bool AdmittanceController::get_bool_param_and_error_if_empty(bool &parameter, const char *parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).get_value<bool>();
        return false; // TODO(destogl): how to check "if_empty" for bool?
    }

    bool AdmittanceController::get_double_param_and_error_if_empty(double &parameter, const char *parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).get_value<double>();
        return false; // TODO: how to check "if_empty" for bool?
    }

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        admittance_controller::AdmittanceController, controller_interface::ControllerInterface)
