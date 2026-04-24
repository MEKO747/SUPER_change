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

#include "sentry_decision/sentry_fsm.h"
#include <cmath>

namespace sentry_fsm {

void SentryFsm::callMainFsmOnce(double current_time) {
    if (!started_) {
        state_ = EMER_STOP;
        return;
    }

    last_state_ = state_;

    switch (state_) {
        case PATROL:
            executePatrol(current_time);
            break;
        case TRACK:
            executeTrack();
            break;
        case ATTACK:
            executeAttack(current_time);
            break;
        case DEFEND:
            executeDefend();
            break;
        case EMER_STOP:
            GimbalCommand cmd;
            cmd.pan = current_gimbal_angle_(0);
            cmd.tilt = current_gimbal_angle_(1);
            cmd.fire = false;
            publishGimbalCommand(cmd);
            break;
    }

    if (last_state_ != state_) {
        fmt::print(fmt::fg(fmt::color::cyan),
                   " [SentryFsm] State transition: {} -> {}\n",
                   SENTRY_STATE_STR[last_state_], SENTRY_STATE_STR[state_]);
    }
}

void SentryFsm::changeState(const std::string& caller, SENTRY_STATE new_state) {
    if (state_ != new_state) {
        state_ = new_state;
    }
}

void SentryFsm::executePatrol(double current_time) {
    // Simple sinusoidal patrol pattern
    double elapsed = current_time - system_start_time_;
    double pan_range = patrol_yaw_max_ - patrol_yaw_min_;
    double pan_center = (patrol_yaw_max_ + patrol_yaw_min_) / 2.0;

    target_gimbal_angle_(0) = pan_center + 0.5 * pan_range * std::sin(elapsed * patrol_speed_);
    target_gimbal_angle_(1) = patrol_tilt_;

    GimbalCommand cmd;
    cmd.pan = target_gimbal_angle_(0);
    cmd.tilt = target_gimbal_angle_(1);
    cmd.fire = false;
    publishGimbalCommand(cmd);

    // Check if target detected - transition to TRACK
    if (current_target_.is_valid && current_target_.distance < max_track_distance_) {
        changeState("Patrol->Track", TRACK);
    }
}

void SentryFsm::executeTrack() {
    if (!current_target_.is_valid) {
        // Lost target, back to patrol
        changeState("Track->Patrol", PATROL);
        return;
    }

    target_gimbal_angle_(0) = current_target_.gimbal_angle(0);
    target_gimbal_angle_(1) = current_target_.gimbal_angle(1);

    // Check if gimbal error is small enough to attack
    double gimbal_error = (current_gimbal_angle_ - target_gimbal_angle_).norm();

    if (gimbal_error < track_lock_threshold_ && current_target_.distance < max_track_distance_) {
        changeState("Track->Attack", ATTACK);
    } else if (gimbal_error > track_lost_threshold_) {
        changeState("Track->Patrol", PATROL);
    }

    GimbalCommand cmd;
    cmd.pan = target_gimbal_angle_(0);
    cmd.tilt = target_gimbal_angle_(1);
    cmd.fire = false;
    publishGimbalCommand(cmd);
}

void SentryFsm::executeAttack(double current_time) {
    if (!current_target_.is_valid || current_target_.distance > max_track_distance_) {
        changeState("Attack->Patrol", PATROL);
        return;
    }

    target_gimbal_angle_(0) = current_target_.gimbal_angle(0);
    target_gimbal_angle_(1) = current_target_.gimbal_angle(1);

    double gimbal_error = (current_gimbal_angle_ - target_gimbal_angle_).norm();
    if (gimbal_error > track_lock_threshold_) {
        changeState("Attack->Track", TRACK);
        return;
    }

    GimbalCommand cmd;
    cmd.pan = target_gimbal_angle_(0);
    cmd.tilt = target_gimbal_angle_(1);
    cmd.fire = shouldFire();
    cmd.fire_rate = cmd.fire ? fire_rate_burst_ : 0.0;
    last_fire_time_ = current_time;

    publishGimbalCommand(cmd);
    publishTargetInfo(current_target_);

    // Could transition to defend based on战术 decision
}

bool SentryFsm::shouldFire() const {
    if (state_ != ATTACK) return false;

    double gimbal_error = (current_gimbal_angle_ - target_gimbal_angle_).norm();
    if (gimbal_error > track_lock_threshold_) return false;
    if (current_target_.distance > max_track_distance_) return false;

    return true;
}

void SentryFsm::executeDefend() {
    // Point gimbal to defense angle (typically away from enemy)
    target_gimbal_angle_(0) = defend_angle_;
    target_gimbal_angle_(1) = 0.0;

    GimbalCommand cmd;
    cmd.pan = target_gimbal_angle_(0);
    cmd.tilt = target_gimbal_angle_(1);
    cmd.fire = false;
    publishGimbalCommand(cmd);
}

double SentryFsm::calculateThreatLevel(const TargetInfo& target) const {
    // Simple threat calculation based on distance and approach speed
    double distance_threat = 1.0 / (target.distance + 0.1);
    double health_threat = target.health / 100.0;
    return distance_threat * 0.7 + health_threat * 0.3;
}

}  // namespace sentry_fsm
