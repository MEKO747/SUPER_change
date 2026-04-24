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

#include "sentry_decision/target_decider.h"
#include <cmath>

namespace sentry_decision {

bool TargetDecider::initialize() {
    return true;
}

int TargetDecider::selectTarget(
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<double>& target_health,
    const std::vector<double>& target_confidence) {

    if (target_positions.empty()) return -1;

    int best_idx = -1;
    double best_score = -1.0;

    for (size_t i = 0; i < target_positions.size(); ++i) {
        double dist = target_positions[i].norm();
        double score = calculatePriorityScore(dist, target_health[i], target_confidence[i]);

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}

AttackDecision TargetDecider::makeAttackDecision(
    double target_distance,
    double gimbal_error,
    double target_valid_time) {

    AttackDecision decision;

    // Check if conditions are met for firing
    if (target_distance > max_fire_distance_) {
        decision.should_fire = false;
        decision.fire_mode = FireMode::STOP;
        return decision;
    }

    if (gimbal_error > max_gimbal_error_) {
        decision.should_fire = false;
        decision.fire_mode = FireMode::STOP;
        return decision;
    }

    if (target_valid_time < min_target_valid_time_) {
        decision.should_fire = false;
        decision.fire_mode = FireMode::STOP;
        return decision;
    }

    // Determine fire mode based on timing
    decision.should_fire = true;
    decision.confidence = 1.0 - (gimbal_error / max_gimbal_error_);

    return decision;
}

Eigen::Vector2d TargetDecider::calculateGimbalAngle(
    const Eigen::Vector3d& target_world_pos,
    const Eigen::Vector3d& sentry_pos,
    const Eigen::Vector2d& sentry_gimbal_offset) {

    Eigen::Vector3d relative_pos = target_world_pos - sentry_pos;

    // Calculate pan (yaw) angle
    double pan = std::atan2(relative_pos.y(), relative_pos.x()) + sentry_gimbal_offset(0);

    // Calculate tilt (pitch) angle
    double horizontal_dist = std::sqrt(relative_pos.x() * relative_pos.x() +
                                        relative_pos.y() * relative_pos.y());
    double tilt = std::atan2(-relative_pos.z(), horizontal_dist) + sentry_gimbal_offset(1);

    return Eigen::Vector2d(pan, tilt);
}

double TargetDecider::calculatePriorityScore(
    double distance,
    double health,
    double confidence) const {

    // Normalize distance (closer = higher score)
    double dist_score = 1.0 / (distance + 1.0);

    // Health score (lower health = higher priority - easier to destroy)
    double health_score = (100.0 - health) / 100.0;

    // Confidence score
    double conf_score = confidence;

    return weight_distance_ * dist_score +
           weight_health_ * health_score +
           weight_confidence_ * conf_score;
}

}  // namespace sentry_decision
