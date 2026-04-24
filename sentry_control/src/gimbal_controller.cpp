/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include "sentry_control/gimbal_controller.h"
#include <cmath>

namespace sentry_control {

bool GimbalController::initialize() {
    return true;
}

void GimbalController::updateState(const GimbalState& state) {
    state_ = state;
}

GimbalControlCommand GimbalController::computeCommand(
    const Eigen::Vector2d& target_angle,
    double dt) {

    GimbalControlCommand cmd;

    // Compute pan error and PID
    double pan_error = target_angle(0) - state_.pan;
    double pan_torque = pan_pid_.compute(pan_error, dt);

    // Compute tilt error and PID
    double tilt_error = target_angle(1) - state_.tilt;
    double tilt_torque = tilt_pid_.compute(tilt_error, dt);

    // Clamp torques
    cmd.pan_torque = std::clamp(pan_torque, -max_torque_, max_torque_);
    cmd.tilt_torque = std::clamp(tilt_torque, -max_torque_, max_torque_);

    // Update reference angles
    state_.pan_ref = target_angle(0);
    state_.tilt_ref = target_angle(1);

    return cmd;
}

bool FireController::initialize() {
    current_friction_speed_ = 0.0;
    dart_feeder_speed_ = 0.0;
    firing_ = false;
    return true;
}

double FireController::controlFrictionWheel(double target_speed) {
    // Simple P control for friction wheel
    double error = target_speed - current_friction_speed_;
    current_friction_speed_ += error * 0.1;  // P gain
    return current_friction_speed_;
}

bool FireController::triggerFire(double friction_speed) {
    if (current_friction_speed_ < friction_wheel_target_ * 0.8) {
        return false;  // Not up to speed
    }

    firing_ = true;
    return true;
}

double FireController::controlDartFeeder(double speed) {
    dart_feeder_speed_ = speed;
    return speed;
}

bool FireController::isReadyToFire() const {
    return std::abs(current_friction_speed_ - friction_wheel_target_) <
           friction_wheel_target_ * 0.1;
}

}  // namespace sentry_control
