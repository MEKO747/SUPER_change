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

#pragma once

#include <memory>
#include <Eigen/Dense>

namespace sentry_control {

struct GimbalState {
    double pan;       // Current pan angle (rad)
    double tilt;      // Current tilt angle (rad)
    double pan_vel;   // Pan angular velocity (rad/s)
    double tilt_vel;  // Tilt angular velocity (rad/s)
    double pan_ref;   // Pan reference angle (rad)
    double tilt_ref;  // Tilt reference angle (rad)

    GimbalState() : pan(0.0), tilt(0.0), pan_vel(0.0), tilt_vel(0.0),
                    pan_ref(0.0), tilt_ref(0.0) {}
};

struct GimbalControlCommand {
    double pan_torque;
    double tilt_torque;
    bool fire_command;
    double friction_wheel_speed;  // RPM

    GimbalControlCommand() : pan_torque(0.0), tilt_torque(0.0),
                              fire_command(false), friction_wheel_speed(0.0) {}
};

class PidController {
public:
    PidController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

    double compute(double error, double dt) {
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() { integral_ = 0.0; prev_error_ = 0.0; }

private:
    double kp_, ki_, kd_;
    double integral_;
    double prev_error_;
};

class GimbalController {
public:
    using Ptr = std::shared_ptr<GimbalController>;

    GimbalController() = default;
    virtual ~GimbalController() = default;

    bool initialize();

    // Update current gimbal state from sensors
    void updateState(const GimbalState& state);

    // Compute control command to track target angle
    GimbalControlCommand computeCommand(const Eigen::Vector2d& target_angle,
                                         double dt);

    // Set PID gains
    void setPanPid(double kp, double ki, double kd) {
        pan_pid_ = PidController(kp, ki, kd);
    }
    void setTiltPid(double kp, double ki, double kd) {
        tilt_pid_ = PidController(kp, ki, kd);
    }

    // Gimbal limits
    void setPanLimits(double min, double max) {
        pan_min_ = min; pan_max_ = max;
    }
    void setTiltLimits(double min, double max) {
        tilt_min_ = min; tilt_max_ = max;
    }

    GimbalState getState() const { return state_; }

private:
    GimbalState state_;
    PidController pan_pid_{2.0, 0.0, 0.1};
    PidController tilt_pid_{2.0, 0.0, 0.1};

    double pan_min_{-M_PI_2}, pan_max_{M_PI_2};
    double tilt_min_{-M_PI_3}, tilt_max_{M_PI_3};

    double max_torque_{10.0};  // N·m
    double max_pan_vel_{5.0};  // rad/s
    double max_tilt_vel_{5.0};  // rad/s
};

class FireController {
public:
    using Ptr = std::shared_ptr<FireController>;

    FireController() = default;
    virtual ~FireController() = default;

    bool initialize();

    // Control friction wheel speed (RPM)
    double controlFrictionWheel(double target_speed);

    // Trigger firing
    bool triggerFire(double friction_speed);

    // Control dart feeder
    double controlDartFeeder(double speed);

    // Check if ready to fire
    bool isReadyToFire() const;

private:
    double current_friction_speed_{0.0};
    double dart_feeder_speed_{0.0};
    bool firing_{false};

    double friction_wheel_target_{3000.0};  // RPM
    double dart_per_shot_{1.0};
    double last_fire_time_{0.0};
};

}  // namespace sentry_control
