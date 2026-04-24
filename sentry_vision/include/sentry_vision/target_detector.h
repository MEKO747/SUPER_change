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
#include <opencv2/opencv.hpp>

namespace sentry_vision {

struct DetectedTarget {
    int id;
    Eigen::Vector2d image_point;     // 2D image coordinates
    Eigen::Vector3d camera_frame_pos; // 3D position in camera frame
    Eigen::Vector3d world_frame_pos;  // 3D position in world frame
    cv::Rect bounding_box;
    double confidence;
    double last_seen_time;

    DetectedTarget() : id(-1), confidence(0.0), last_seen_time(0.0) {}
};

class TargetDetector {
public:
    using Ptr = std::shared_ptr<TargetDetector>;

    TargetDetector() = default;
    virtual ~TargetDetector() = default;

    virtual bool initialize() = 0;
    virtual std::vector<DetectedTarget> detect(const cv::Mat& image) = 0;
    virtual bool loadModel(const std::string& model_path) = 0;

    void setCameraIntrinsics(const Eigen::Matrix3d& K) { K_ = K; }
    void setCameraExtrinsics(const Eigen::Matrix4d& T_cam_to_world) {
        T_cam_to_world_ = T_cam_to_world;
    }

protected:
    Eigen::Matrix3d K_;                    // Camera intrinsic matrix
    Eigen::Matrix4d T_cam_to_world_;       // Camera to world transformation
    bool initialized_{false};
};

class KalmanTracker {
public:
    using Ptr = std::shared_ptr<KalmanTracker>;

    KalmanTracker(int state_dim = 6, int measure_dim = 3);

    void predict(double dt);
    void update(const Eigen::Vector3d& measurement);

    Eigen::Vector3d getState() const { return state_.head<3>(); }
    Eigen::Vector3d getVelocity() const { return state_.tail<3>(); }
    double getDistance(const Eigen::Vector3d& pos) const;

    bool isTrackValid(double max_distance, double max_time) const;
    void setTrackId(int id) { track_id_ = id; }
    int getTrackId() const { return track_id_; }

private:
    int track_id_{-1};
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;  // Covariance
    Eigen::MatrixXd Q_;   // Process noise
    Eigen::MatrixXd R_;   // Measurement noise
    double last_update_time_{0.0};
};

class TargetTracker {
public:
    using Ptr = std::shared_ptr<TargetTracker>;

    TargetTracker() = default;
    virtual ~TargetTracker() = default;

    bool initialize();
    std::vector<DetectedTarget> track(const std::vector<DetectedTarget>& detections,
                                       double current_time);

    void setMaxTrackDistance(double d) { max_track_distance_ = d; }
    void setMaxTrackTime(double t) { max_track_time_ = t; }

private:
    std::vector<KalmanTracker::Ptr> trackers_;
    std::vector<int> tracker_ids_;
    int next_track_id_{0};
    double max_track_distance_{10.0};
    double max_track_time_{2.0};

    int matchDetectionsToTrackers(const std::vector<DetectedTarget>& detections);
    void removeLostTrackers(double current_time);
    void addNewTracker(const DetectedTarget& detection);
};

}  // namespace sentry_vision
