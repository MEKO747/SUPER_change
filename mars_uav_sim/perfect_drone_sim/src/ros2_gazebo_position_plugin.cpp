#include <atomic>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
class SuperGazeboPositionPlugin : public ModelPlugin {
public:
  SuperGazeboPositionPlugin() = default;

  ~SuperGazeboPositionPlugin() override {
    running_.store(false);
    if (update_connection_) {
      update_connection_.reset();
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    if (sdf->HasElement("world_frame")) {
      world_frame_ = sdf->Get<std::string>("world_frame");
    }
    if (sdf->HasElement("base_frame")) {
      base_frame_ = sdf->Get<std::string>("base_frame");
    }
    if (sdf->HasElement("cmd_topic")) {
      cmd_topic_ = sdf->Get<std::string>("cmd_topic");
    }
    if (sdf->HasElement("odom_topic")) {
      odom_topic_ = sdf->Get<std::string>("odom_topic");
    }
    if (sdf->HasElement("pose_topic")) {
      pose_topic_ = sdf->Get<std::string>("pose_topic");
    }
    if (sdf->HasElement("publish_rate")) {
      publish_period_ = 1.0 / std::max(1.0, sdf->Get<double>("publish_rate"));
    }
    if (sdf->HasElement("command_timeout")) {
      command_timeout_ = sdf->Get<double>("command_timeout");
    }

    pose_ = model_->WorldPose();
    velocity_.setZero();

    const rclcpp::QoS qos(rclcpp::QoS(100).best_effort().keep_last(100).durability_volatile());
    cmd_sub_ = node_->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        cmd_topic_, qos, std::bind(&SuperGazeboPositionPlugin::cmdCallback, this, std::placeholders::_1));
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SuperGazeboPositionPlugin::onUpdate, this));

    running_.store(true);
    RCLCPP_INFO(node_->get_logger(),
                "SUPER Gazebo position bridge loaded: %s -> %s, %s",
                cmd_topic_.c_str(), odom_topic_.c_str(), pose_topic_.c_str());
  }

private:
  void cmdCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
    const Eigen::Vector3d velocity(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    const Eigen::Vector3d acceleration(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    const double yaw = msg->yaw;

    const Eigen::Vector3d gravity(0.0, 0.0, 9.80);
    const Eigen::Vector3d zb = (gravity + acceleration).normalized();
    const Eigen::Vector3d xc(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d yb = zb.cross(xc);
    if (yb.norm() < 1e-6) {
      yb = Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0.0);
    }
    yb.normalize();
    const Eigen::Vector3d xb = yb.cross(zb).normalized();

    Eigen::Matrix3d rotation;
    rotation << xb, yb, zb;
    const Eigen::Quaterniond q(rotation);

    pose_.Pos().Set(position.x(), position.y(), position.z());
    pose_.Rot().Set(q.w(), q.x(), q.y(), q.z());
    velocity_ = velocity;
    last_command_time_ = world_->SimTime().Double();
    has_command_ = true;
  }

  void onUpdate() {
    if (!running_.load()) {
      return;
    }

    const double sim_time = world_->SimTime().Double();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_command_ && sim_time - last_command_time_ <= command_timeout_) {
        model_->SetWorldPose(pose_);
        model_->SetLinearVel(ignition::math::Vector3d(velocity_.x(), velocity_.y(), velocity_.z()));
      } else {
        pose_ = model_->WorldPose();
        const auto vel = model_->WorldLinearVel();
        velocity_ = Eigen::Vector3d(vel.X(), vel.Y(), vel.Z());
      }
    }

    if (sim_time - last_publish_time_ < publish_period_) {
      return;
    }
    last_publish_time_ = sim_time;
    publishState();
  }

  void publishState() {
    ignition::math::Pose3d pose;
    Eigen::Vector3d velocity;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = pose_;
      velocity = velocity_;
    }

    const auto stamp = node_->get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mutex_;
  ignition::math::Pose3d pose_;
  Eigen::Vector3d velocity_{0.0, 0.0, 0.0};
  bool has_command_{false};
  double last_command_time_{0.0};
  double last_publish_time_{0.0};
  double publish_period_{0.01};
  double command_timeout_{0.5};
  std::atomic_bool running_{false};

  std::string world_frame_{"world"};
  std::string base_frame_{"base_link"};
  std::string cmd_topic_{"/planning/pos_cmd"};
  std::string odom_topic_{"/lidar_slam/odom"};
  std::string pose_topic_{"/lidar_slam/pose"};
};

GZ_REGISTER_MODEL_PLUGIN(SuperGazeboPositionPlugin)
}  // namespace gazebo
