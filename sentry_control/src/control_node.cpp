/**
 * Sentry Control Node
 * ROS node for gimbal control and fire mechanism
 */

#include <ros/ros.h>
#include "sentry_control/gimbal_controller.h"
#include "sentry_msgs/GimbalCommand.h"
#include "sentry_msgs/GimbalState.h"
#include "sentry_msgs/FireCommand.h"
#include "sentry_msgs/GimbalState.h"

class ControlNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gimbal_cmd_sub_;
    ros::Publisher gimbal_state_pub_;
    ros::Publisher fire_cmd_pub_;
    ros::Timer control_timer_;

    sentry_control::GimbalController::Ptr gimbal_ctrl_;
    sentry_control::FireController::Ptr fire_ctrl_;

    sentry_control::GimbalState current_state_;
    Eigen::Vector2d target_angle_{0.0, 0.0};
    bool fire_command_{false};

public:
    ControlNode() {
        // Initialize controllers
        gimbal_ctrl_ = std::make_shared<sentry_control::GimbalController>();
        gimbal_ctrl_->initialize();

        fire_ctrl_ = std::make_shared<sentry_control::FireController>();
        fire_ctrl_->initialize();

        // Load PID parameters from ROS param server
        loadParams();

        // Subscribe to gimbal commands from decision node
        gimbal_cmd_sub_ = nh_.subscribe("/sentry/gimbal_cmd", 1,
                                          &ControlNode::gimbalCmdCallback, this);

        // Publish gimbal state feedback
        gimbal_state_pub_ = nh_.advertise<sentry_msgs::GimbalState>("/sentry/gimbal_state", 10);

        // Publish fire commands
        fire_cmd_pub_ = nh_.advertise<sentry_msgs::FireCommand>("/sentry/fire_cmd", 10);

        // Control loop at 200Hz
        control_timer_ = nh_.createTimer(ros::Duration(0.005),
                                          &ControlNode::controlCallback, this);

        ROS_INFO("Sentry Control Node initialized");
    }

    void loadParams() {
        double pan_kp, pan_ki, pan_kd;
        double tilt_kp, tilt_ki, tilt_kd;

        nh_.param("gimbal/pan_kp", pan_kp, 2.0);
        nh_.param("gimbal/pan_ki", pan_ki, 0.0);
        nh_.param("gimbal/pan_kd", pan_kd, 0.1);
        nh_.param("gimbal/tilt_kp", tilt_kp, 2.0);
        nh_.param("gimbal/tilt_ki", tilt_ki, 0.0);
        nh_.param("gimbal/tilt_kd", tilt_kd, 0.1);

        gimbal_ctrl_->setPanPid(pan_kp, pan_ki, pan_kd);
        gimbal_ctrl_->setTiltPid(tilt_kp, tilt_ki, tilt_kd);

        double pan_min, pan_max, tilt_min, tilt_max;
        nh_.param("gimbal/pan_min", pan_min, -1.57);
        nh_.param("gimbal/pan_max", pan_max, 1.57);
        nh_.param("gimbal/tilt_min", tilt_min, -1.05);
        nh_.param("gimbal/tilt_max", tilt_max, 0.52);

        gimbal_ctrl_->setPanLimits(pan_min, pan_max);
        gimbal_ctrl_->setTiltLimits(tilt_min, tilt_max);
    }

    void gimbalCmdCallback(const sentry_msgs::GimbalCommand::ConstPtr& msg) {
        target_angle_ << msg->pan_target, msg->tilt_target;
        fire_command_ = msg->fire;
    }

    void controlCallback(const ros::TimerEvent&) {
        double dt = 0.005;

        // Get current state from sensors (simulated here, would come from CAN/encoders)
        // updateStateFromSensors();

        // Compute control command
        auto cmd = gimbal_ctrl_->computeCommand(target_angle_, dt);

        // Publish gimbal state
        sentry_msgs::GimbalState state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.pan_current = current_state_.pan;
        state_msg.tilt_current = current_state_.tilt;
        state_msg.pan_velocity = current_state_.pan_vel;
        state_msg.tilt_velocity = current_state_.tilt_vel;
        gimbal_state_pub_.publish(state_msg);

        // Handle fire command
        if (fire_command_) {
            sentry_msgs::FireCommand fire_msg;
            fire_msg.header.stamp = ros::Time::now();
            fire_msg.fire = true;
            fire_msg.friction_wheel_rpm = 3000.0;
            fire_msg.dart_feeder_speed = 10.0;
            fire_cmd_pub_.publish(fire_msg);
        }

        // In real implementation, send cmd.pan_torque and cmd.tilt_torque via CAN
        // sendCanCommand(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sentry_control_node");
    ControlNode node;
    ros::spin();
    return 0;
}
