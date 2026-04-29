#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class WorldCloudTransformer : public rclcpp::Node {
public:
  WorldCloudTransformer() : Node("world_cloud_transformer"), buffer_(this->get_clock()), listener_(buffer_) {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "world");
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/gazebo/lidar/points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cloud_registered");

    const rclcpp::QoS pub_qos(rclcpp::QoS(10).reliable().keep_last(10).durability_volatile());
    const rclcpp::QoS sub_qos(rclcpp::QoS(10).best_effort().keep_last(10).durability_volatile());
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, sub_qos, std::bind(&WorldCloudTransformer::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Transforming %s to %s in frame %s",
                input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = buffer_.lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Skipping cloud transform %s -> %s: %s",
                           msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;
    out.header.stamp = this->get_clock()->now();
    out.header.frame_id = target_frame_;

    const auto &t = tf_msg.transform.translation;
    const auto &q = tf_msg.transform.rotation;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    const double r00 = 1.0 - 2.0 * (yy + zz);
    const double r01 = 2.0 * (xy - wz);
    const double r02 = 2.0 * (xz + wy);
    const double r10 = 2.0 * (xy + wz);
    const double r11 = 1.0 - 2.0 * (xx + zz);
    const double r12 = 2.0 * (yz - wx);
    const double r20 = 2.0 * (xz - wy);
    const double r21 = 2.0 * (yz + wx);
    const double r22 = 1.0 - 2.0 * (xx + yy);

    sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      *iter_x = static_cast<float>(r00 * x + r01 * y + r02 * z + t.x);
      *iter_y = static_cast<float>(r10 * x + r11 * y + r12 * z + t.y);
      *iter_z = static_cast<float>(r20 * x + r21 * y + r22 * z + t.z);
    }

    cloud_pub_->publish(out);
  }

  std::string target_frame_;
  std::string input_topic_;
  std::string output_topic_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
