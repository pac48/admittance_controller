//
// Created by paul on 5/31/22.
//
#include "admittance_joint_limits/admittance_joint_limits.hpp"
#include "joint_limits_interface/joint_limits.hpp"


admittance_joint_limits::AdmittanceJointLimits::AdmittanceJointLimits(const transmission_interface::JointHandle &jposh,
                                                                      const transmission_interface::JointHandle &jcmdh,
                                                                      const joint_limits_interface::JointLimits &limits)
        : JointLimitHandle(jposh, jcmdh, limits) {

    if (limits_.has_position_limits)
    {
        min_pos_limit_ = limits_.min_position;
        max_pos_limit_ = limits_.max_position;
    }
    else
    {
        min_pos_limit_ = -std::numeric_limits<double>::max();
        max_pos_limit_ = std::numeric_limits<double>::max();
    }

}


void admittance_joint_limits::AdmittanceJointLimits::enforce_limits(const rclcpp::Duration &period) {
    if (std::isnan(prev_pos_))
    {
        prev_pos_ = jposh_.get_value();
    }

    double min_pos, max_pos;
    if (limits_.has_velocity_limits)
    {
        // enforce velocity limits
        // set constraints on where the position can be based on the
        // max velocity times seconds since last update
        const double delta_pos = limits_.max_velocity * period.seconds();
        min_pos = std::max(prev_pos_ - delta_pos, min_pos_limit_);
        max_pos = std::min(prev_pos_ + delta_pos, max_pos_limit_);
    }
    else
    {
        // no velocity limit, so position is simply limited to set extents (our imposed soft limits)
        min_pos = min_pos_limit_;
        max_pos = max_pos_limit_;
    }

    // clamp command position to our computed min/max position
    const double cmd = rcppmath::clamp(jcmdh_.get_value(), min_pos, max_pos);
    jcmdh_.set_value(cmd);

    prev_pos_ = cmd;
}

