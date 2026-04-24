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

#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace sentry_decision {

enum class FireMode {
    SINGLE,
    BURST,
    STOP
};

struct AttackDecision {
    bool should_fire;
    FireMode fire_mode;
    double confidence;

    AttackDecision() : should_fire(false), fire_mode(FireMode::STOP), confidence(0.0) {}
};

struct SentryStatus {
    double gimbal_pan;
    double gimbal_tilt;
    double fire_ready;
    double cooldown_remaining;
    int targets_in_view;

    SentryStatus() : gimbal_pan(0.0), gimbal_tilt(0.0),
                     fire_ready(1.0), cooldown_remaining(0.0), targets_in_view(0) {}
};

class TargetDecider {
public:
    using Ptr = std::shared_ptr<TargetDecider>;

    TargetDecider() = default;
    virtual ~TargetDecider() = default;

    bool initialize();

    // Select target from available targets based on priority
    int selectTarget(const std::vector<Eigen::Vector3d>& target_positions,
                     const std::vector<double>& target_health,
                     const std::vector<double>& target_confidence);

    // Make attack decision
    AttackDecision makeAttackDecision(double target_distance,
                                       double gimbal_error,
                                       double target_valid_time);

    // Calculate gimbal angle to point at target
    Eigen::Vector2d calculateGimbalAngle(const Eigen::Vector3d& target_world_pos,
                                          const Eigen::Vector3d& sentry_pos,
                                          const Eigen::Vector2d& sentry_gimbal_offset);

private:
    // Priority weights
    double weight_distance_{0.4};
    double weight_health_{0.3};
    double weight_confidence_{0.3};

    // Attack thresholds
    double max_fire_distance_{8.0};
    double max_gimbal_error_{0.15};  // rad
    double min_target_valid_time_{0.3};

    // Fire timing
    double burst_interval_{0.2};  // seconds between burst shots
    double single_cooldown_{0.5};

    double last_burst_time_{0.0};
    FireMode last_fire_mode_{FireMode::STOP};

    double calculatePriorityScore(double distance, double health,
                                   double confidence) const;
};

}  // namespace sentry_decision
