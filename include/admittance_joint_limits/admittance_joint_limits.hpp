//
// Created by paul on 5/31/22.
//

#ifndef ADMITTANCE_WS_ADMITTANCE_JOINT_LIMITS_H
#define ADMITTANCE_WS_ADMITTANCE_JOINT_LIMITS_H

#include "joint_limits_interface/joint_limits_interface.hpp"
#include <urdf/urdfdom_compatibility.h>
#include <urdf_model/joint.h>
#include <joint_limits_interface/joint_limits.hpp>

namespace admittance_joint_limits
{
    class AdmittanceJointLimits : public joint_limits_interface::JointLimitHandle{
    AdmittanceJointLimits(
            const transmission_interface::JointHandle & jposh, const transmission_interface::JointHandle & jcmdh,
            const joint_limits_interface::JointLimits & limits);
    public:
        void enforce_limits(const rclcpp::Duration &period) override;

    private:
        double min_pos_limit_ = 0.0;
        double max_pos_limit_ = 0.0;
    };



}


#endif //ADMITTANCE_WS_ADMITTANCE_JOINT_LIMITS_H
