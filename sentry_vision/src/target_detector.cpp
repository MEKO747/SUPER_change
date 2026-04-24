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

#include "sentry_vision/target_detector.h"
#include <Eigen/src/Geometry/Transform.h>

namespace sentry_vision {

KalmanTracker::KalmanTracker(int state_dim, int measure_dim)
    : state_(state_dim), P_(state_dim, state_dim),
      Q_(state_dim, state_dim), R_(measure_dim, measure_dim) {

    state_.setZero();
    P_.setIdentity();
    Q_.setIdentity() *= 0.01;
    R_.setIdentity() *= 0.1;
}

void KalmanTracker::predict(double dt) {
    // State transition: x = [pos, vel], predict using constant velocity model
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_.size(), state_.size());
    F(0, 3) = dt;  // x += vx * dt
    F(1, 4) = dt;  // y += vy * dt
    F(2, 5) = dt;  // z += vz * dt

    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanTracker::update(const Eigen::Vector3d& measurement) {
    // Measurement update
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state_.size());
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    Eigen::VectorXd z(3);
    z << measurement;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    state_ = state_ + K * (z - H * state_);
    P_ = (Eigen::MatrixXd::Identity(state_.size(), state_.size()) - K * H) * P_;
}

double KalmanTracker::getDistance(const Eigen::Vector3d& pos) const {
    return (state_.head<3>() - pos).norm();
}

bool KalmanTracker::isTrackValid(double max_distance, double max_time) const {
    return state_.head<3>().norm() < max_distance;
}

bool TargetTracker::initialize() {
    trackers_.clear();
    tracker_ids_.clear();
    next_track_id_ = 0;
    return true;
}

std::vector<DetectedTarget> TargetTracker::track(
    const std::vector<DetectedTarget>& detections,
    double current_time) {

    // Match existing trackers to detections
    matchDetectionsToTrackers(detections);

    // Remove lost trackers
    removeLostTrackers(current_time);

    // Add new detections as new trackers
    for (const auto& detection : detections) {
        bool matched = false;
        for (const auto& id : tracker_ids_) {
            if (id >= 0) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            addNewTracker(detection);
        }
    }

    // Return tracked targets
    std::vector<DetectedTarget> tracked;
    for (size_t i = 0; i < trackers_.size(); ++i) {
        if (trackers_[i] && isTrackValid(max_track_distance_, max_track_time_)) {
            DetectedTarget t;
            t.id = tracker_ids_[i];
            t.world_frame_pos = trackers_[i]->getState();
            tracked.push_back(t);
        }
    }
    return tracked;
}

int TargetTracker::matchDetectionsToTrackers(
    const std::vector<DetectedTarget>& detections) {

    // Simple nearest neighbor matching
    for (const auto& detection : detections) {
        double min_dist = std::numeric_limits<double>::max();
        int best_idx = -1;

        for (size_t i = 0; i < trackers_.size(); ++i) {
            if (!trackers_[i]) continue;

            double dist = trackers_[i]->getDistance(detection.world_frame_pos);
            if (dist < min_dist && dist < max_track_distance_) {
                min_dist = dist;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            trackers_[best_idx]->update(detection.world_frame_pos);
            tracker_ids_[best_idx] = detection.id >= 0 ? detection.id : tracker_ids_[best_idx];
        }
    }

    return 0;
}

void TargetTracker::removeLostTrackers(double current_time) {
    for (auto it = trackers_.begin(); it != trackers_.end(); ) {
        size_t idx = std::distance(trackers_.begin(), it);
        if (*it && !(*it)->isTrackValid(max_track_distance_, max_track_time_)) {
            trackers_.erase(it);
            tracker_ids_.erase(tracker_ids_.begin() + idx);
        } else {
            ++it;
        }
    }
}

void TargetTracker::addNewTracker(const DetectedTarget& detection) {
    auto tracker = std::make_shared<KalmanTracker>(6, 3);
    tracker->setTrackId(next_track_id_++);
    tracker->update(detection.world_frame_pos);

    trackers_.push_back(tracker);
    tracker_ids_.push_back(tracker->getTrackId());
}

}  // namespace sentry_vision
