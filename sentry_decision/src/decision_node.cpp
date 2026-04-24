/**
 * Sentry Decision Node
 * ROS node for target prioritization and attack decisions
 */

#include <ros/ros.h>
#include <ros/topic.h>
#include "sentry_decision/sentry_fsm.h"
#include "sentry_decision/target_decider.h"
#include "sentry_msgs/TargetInfo.h"
#include "sentry_msgs/GimbalCommand.h"
#include "sentry_msgs/GimbalState.h"
#include "sentry_msgs/SentryStatus.h"

class DecisionNode : public sentry_fsm::SentryFsm {
private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Subscriber gimbal_state_sub_;
    ros::Publisher gimbal_cmd_pub_;
    ros::Publisher sentry_status_pub_;

    sentry_decision::TargetDecider::Ptr decider_;
    ros::Timer fsm_timer_;

    std::vector<sentry_fsm::TargetInfo> available_targets_;
    Eigen::Vector2d current_gimbal_angle_{0.0, 0.0};

public:
    DecisionNode() {
        // Initialize decider
        decider_ = std::make_shared<sentry_decision::TargetDecider>();
        decider_->initialize();

        // Subscribe to target detections
        target_sub_ = nh_.subscribe("/sentry/target", 10,
                                     &DecisionNode::targetCallback, this);

        // Subscribe to gimbal state feedback
        gimbal_state_sub_ = nh_.subscribe("/sentry/gimbal_state", 10,
                                           &DecisionNode::gimbalStateCallback, this);

        // Publish gimbal commands
        gimbal_cmd_pub_ = nh_.advertise<sentry_msgs::GimbalCommand>("/sentry/gimbal_cmd", 10);

        // Publish sentry status
        sentry_status_pub_ = nh_.advertise<sentry_msgs::SentryStatus>("/sentry/status", 10);

        // FSM timer at 100Hz
        fsm_timer_ = nh_.createTimer(ros::Duration(0.01),
                                      &DecisionNode::fsmCallback, this);

        // Start FSM
        start(ros::Time::now().toSec());

        ROS_INFO("Sentry Decision Node initialized");
    }

    void targetCallback(const sentry_msgs::TargetInfo::ConstPtr& msg) {
        sentry_fsm::TargetInfo target;
        target.id = msg->id;
        target.position << msg->world_position.x,
                           msg->world_position.y,
                           msg->world_position.z;
        target.distance = msg->distance;
        target.health = msg->health;
        target.threat_level = msg->threat_level;
        target.gimbal_angle << msg->gimbal_pan, msg->gimbal_tilt;
        target.is_valid = msg->is_valid;
        target.last_seen_time = msg->last_seen_time;

        updateTarget(target);
    }

    void gimbalStateCallback(const sentry_msgs::GimbalState::ConstPtr& msg) {
        current_gimbal_angle_ << msg->pan_current, msg->tilt_current;
        updateGimbalAngle(current_gimbal_angle_);
    }

    void fsmCallback(const ros::TimerEvent&) {
        double current_time = ros::Time::now().toSec();
        callMainFsmOnce(current_time);

        // Publish status
        sentry_msgs::SentryStatus status;
        status.header.stamp = ros::Time::now();
        status.state = getState();
        status.state_str = getStateStr();
        status.gimbal_pan = current_gimbal_angle_(0);
        status.gimbal_tilt = current_gimbal_angle_(1);
        status.fire_ready = (getState() == sentry_fsm::ATTACK);
        status.targets_in_view = available_targets_.size();
        sentry_status_pub_.publish(status);
    }

protected:
    void publishGimbalCommand(const sentry_fsm::GimbalCommand& cmd) override {
        sentry_msgs::GimbalCommand msg;
        msg.header.stamp = ros::Time::now();
        msg.pan_target = cmd.pan;
        msg.tilt_target = cmd.tilt;
        msg.fire = cmd.fire;
        msg.fire_rate = cmd.fire_rate;
        gimbal_cmd_pub_.publish(msg);
    }

    void publishTargetInfo(const sentry_fsm::TargetInfo& target) override {
        // Target info already published by vision node
    }

    void resetVisualization() override {
        // Reset visualization markers if needed
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sentry_decision_node");
    DecisionNode node;
    ros::spin();
    return 0;
}
