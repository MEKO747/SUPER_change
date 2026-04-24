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
#include <vector>
#include <Eigen/Dense>
#include <fmt/color.h>

namespace sentry_fsm {

enum SENTRY_STATE {
    PATROL = 0,    // Patrol scan mode - searching for targets
    TRACK = 1,     // Target locked - tracking enemy
    ATTACK = 2,    // Attack mode - firing at target
    DEFEND = 3,    // Defense mode - shield or evade
    EMER_STOP = 4  // Emergency stop
};

static const std::vector<std::string> SENTRY_STATE_STR{
    "PATROL",
    "TRACK",
    "ATTACK",
    "DEFEND",
    "EMER_STOP"
};

struct TargetInfo {
    int id;
    Eigen::Vector3d position;      // World frame position
    Eigen::Vector2d gimbal_angle; // pan, tilt angles to target
    double distance;
    double health;                 // Enemy health percentage
    double threat_level;           // Calculated threat level
    bool is_valid;
    double last_seen_time;

    TargetInfo() : id(-1), distance(0.0), health(100.0),
                   threat_level(0.0), is_valid(false), last_seen_time(0.0) {}
};

struct GimbalCommand {
    double pan;    // Pan angle (yaw)
    double tilt;   // Tilt angle (pitch)
    bool fire;
    double fire_rate; // rounds per second

    GimbalCommand() : pan(0.0), tilt(0.0), fire(false), fire_rate(0.0) {}
};

class SentryFsm {
protected:
    SENTRY_STATE state_{PATROL};
    SENTRY_STATE last_state_{PATROL};

    // Current target being tracked/attacked
    TargetInfo current_target_;

    // Gimbal state
    Eigen::Vector2d current_gimbal_angle_;  // Current pan, tilt
    Eigen::Vector2d target_gimbal_angle_;   // Target pan, tilt

    // Patrol parameters
    double patrol_yaw_min_{-M_PI_4};
    double patrol_yaw_max_{M_PI_4};
    double patrol_tilt_{-M_PI_6};
    double patrol_speed_{0.5};  // rad/s

    // Tracking parameters
    double track_lock_threshold_{0.1};  // rad - angle threshold to lock target
    double track_lost_threshold_{1.0};   // rad - angle threshold to consider target lost
    double max_track_distance_{10.0};   // meters

    // Attack parameters
    double fire_rate_single_{1.0};   // single shot
    double fire_rate_burst_{5.0};    // burst shot
    double fire_cooldown_{0.2};      // seconds between shots
    double last_fire_time_{0.0};

    // Defense parameters
    double defend_angle_{M_PI};  // angle to face for defense

    bool started_{false};
    double system_start_time_{0.0};

    virtual void publishGimbalCommand(const GimbalCommand& cmd) = 0;
    virtual void publishTargetInfo(const TargetInfo& target) = 0;
    virtual void resetVisualization() = 0;

public:
    SentryFsm() = default;
    virtual ~SentryFsm() = default;

    void start(double current_time) {
        system_start_time_ = current_time;
        started_ = true;
        state_ = PATROL;
    }

    void stop() {
        started_ = false;
        state_ = EMER_STOP;
    }

    SENTRY_STATE getState() const { return state_; }
    std::string getStateStr() const { return SENTRY_STATE_STR[state_]; }

    void updateTarget(const TargetInfo& target) {
        current_target_ = target;
    }

    void updateGimbalAngle(const Eigen::Vector2d& angle) {
        current_gimbal_angle_ = angle;
    }

    void setPatrolRange(double yaw_min, double yaw_max, double tilt) {
        patrol_yaw_min_ = yaw_min;
        patrol_yaw_max_ = yaw_max;
        patrol_tilt_ = tilt;
    }

    void callMainFsmOnce(double current_time);

    Eigen::Vector2d getTargetGimbalAngle() const { return target_gimbal_angle_; }
    bool shouldFire() const;

protected:
    void changeState(const std::string& caller, SENTRY_STATE new_state);

    void executePatrol(double current_time);
    void executeTrack();
    void executeAttack(double current_time);
    void executeDefend();

    double calculateThreatLevel(const TargetInfo& target) const;
};

}  // namespace sentry_fsm
