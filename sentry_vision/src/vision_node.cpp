/**
 * Sentry Vision Node
 * ROS node for enemy detection and target tracking
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include "sentry_vision/target_detector.h"
#include "sentry_msgs/GimbalState.h"

class VisionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber gimbal_state_sub_;
    ros::Publisher target_pub_;

    sentry_vision::TargetTracker::Ptr tracker_;
    sentry_vision::TargetDetector::Ptr detector_;

    Eigen::Matrix3d K_;
    Eigen::Matrix4d T_cam_to_world_;
    bool camera_initialized_{false};

    double current_pan_{0.0};
    double current_tilt_{0.0};

public:
    VisionNode() : it_(nh_) {
        // Subscribe to camera image
        image_sub_ = it_.subscribe("/sentry/camera/image", 1,
                                    &VisionNode::imageCallback, this);

        // Subscribe to gimbal state for coordinate transformation
        gimbal_state_sub_ = nh_.subscribe("/sentry/gimbal_state", 1,
                                           &VisionNode::gimbalStateCallback, this);

        // Publish detected targets
        target_pub_ = nh_.advertise<sentry_msgs::TargetInfo>("/sentry/target", 10);

        // Initialize tracker
        tracker_ = std::make_shared<sentry_vision::TargetTracker>();
        tracker_->initialize();

        // Load camera parameters from ROS param server
        loadCameraParams();
    }

    void loadCameraParams() {
        // Camera intrinsic parameters
        double fx, fy, cx, cy;
        nh_.param("camera/fx", fx, 600.0);
        nh_.param("camera/fy", fy, 600.0);
        nh_.param("camera/cx", cx, 320.0);
        nh_.param("camera/cy", cy, 240.0);

        K_ << fx, 0, cx,
              0, fy, cy,
              0, 0, 1;

        // Camera offset from gimbal center
        double cam_x, cam_y, cam_z;
        nh_.param("camera/offset_x", cam_x, 0.1);
        nh_.param("camera/offset_y", cam_y, 0.0);
        nh_.param("camera/offset_z", cam_z, 0.0);

        T_cam_to_world_.setIdentity();
        T_cam_to_world_(0, 3) = cam_x;
        T_cam_to_world_(1, 3) = cam_y;
        T_cam_to_world_(2, 3) = cam_z;

        camera_initialized_ = true;
    }

    void gimbalStateCallback(const sentry_msgs::GimbalState::ConstPtr& msg) {
        current_pan_ = msg->pan_current;
        current_tilt_ = msg->tilt_current;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!camera_initialized_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Process image (detection would be done here with YOLO)
        // For now, just pass through for tracking
        std::vector<sentry_vision::DetectedTarget> detections;

        // Run tracking update
        auto tracked = tracker_->track(detections, ros::Time::now().toSec());

        // Publish tracked targets
        for (const auto& target : tracked) {
            sentry_msgs::TargetInfo info;
            info.header.stamp = msg->header.stamp;
            info.id = target.id;
            info.world_position.x = target.world_frame_pos.x();
            info.world_position.y = target.world_frame_pos.y();
            info.world_position.z = target.world_frame_pos.z();
            info.distance = target.world_frame_pos.norm();
            info.is_valid = true;
            info.last_seen_time = target.last_seen_time;

            // Calculate gimbal angles to target
            double pan = std::atan2(target.world_frame_pos.y(),
                                    target.world_frame_pos.x()) - current_pan_;
            double tilt = std::atan2(-target.world_frame_pos.z(),
                                     std::sqrt(target.world_frame_pos.x() * target.world_frame_pos.x() +
                                               target.world_frame_pos.y() * target.world_frame_pos.y()));

            info.gimbal_pan = pan;
            info.gimbal_tilt = tilt;

            target_pub_.publish(info);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sentry_vision_node");
    VisionNode node;
    ros::spin();
    return 0;
}
