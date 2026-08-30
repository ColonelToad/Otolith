#include "otolith/fusion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("otolith_fusion"), ekf_() {
    // QoS: match sim_node (BEST_EFFORT, depth 1)
    auto qos = rclcpp::QoS(1).best_effort();

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/otolith/imu", qos, std::bind(&FusionNode::on_imu, this, std::placeholders::_1));
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "/otolith/joint_states", qos, std::bind(&FusionNode::on_joints, this, std::placeholders::_1));
    sub_contacts_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/otolith/foot_contacts", qos, std::bind(&FusionNode::on_contacts, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/otolith/state_estimate", qos);

    // watchdogs
    timer_watchdog_ = create_wall_timer(1s, std::bind(&FusionNode::watchdog, this));

    RCLCPP_INFO(get_logger(), "otolith_fusion up: subscribing /otolith/imu|joint_states|foot_contacts -> /otolith/state_estimate");
  }

private:
  void on_joints(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    // JointState name order: FL/R hip,thigh,calf (12) — matches logger qj order
    if (msg->position.size() >= 12) {
      for (int i = 0; i < 12 && i < (int)msg->position.size(); ++i) latest_qj_[i] = msg->position[i];
      has_qj_ = true;
      last_joints_time_ = now();
    }
  }
  void on_contacts(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (msg->data.size() >= 4) {
      for (int i = 0; i < 4; ++i) latest_contacts_[i] = msg->data[i] > 0.5f ? 1 : 0;
      has_contacts_ = true;
      last_contacts_time_ = now();
    }
  }
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // dt from header stamp or fixed 0.002
    double dt = 0.002;
    auto stamp = msg->header.stamp;
    double t = stamp.sec + stamp.nanosec * 1e-9;
    if (has_last_imu_ && t > last_imu_header_time_ + 1e-6 && t < last_imu_header_time_ + 0.1) dt = t - last_imu_header_time_;
    last_imu_header_time_ = t; has_last_imu_ = true;
    last_imu_wall_time_ = now();

    // init from first IMU orientation if not yet
    if (!initialized_) {
      // q from Imu orientation (if valid)
      double n = std::sqrt(msg->orientation.w*msg->orientation.w + msg->orientation.x*msg->orientation.x + msg->orientation.y*msg->orientation.y + msg->orientation.z*msg->orientation.z);
      if (n > 0.1) {
        auto s = ekf_.state();
        s.q = Eigen::Quaterniond(msg->orientation.w/n, msg->orientation.x/n, msg->orientation.y/n, msg->orientation.z/n);
        s.q.normalize();
        ekf_.set_state(s);
      }
      initialized_ = true;
    }

    ekf_.predict(dt, gyro, accel);
    imu_count_++;
    if (has_qj_ && has_contacts_) {
      Eigen::Matrix<double,12,1> qj;
      for (int i = 0; i < 12; ++i) qj[i] = latest_qj_[i];
      std::array<uint8_t,4> contacts{latest_contacts_[0], latest_contacts_[1], latest_contacts_[2], latest_contacts_[3]};
      ekf_.update_legs(qj, contacts, gyro, dt);
    }

    // publish
    auto st = ekf_.state();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base";
    odom.pose.pose.position.x = st.p.x(); odom.pose.pose.position.y = st.p.y(); odom.pose.pose.position.z = st.p.z();
    odom.pose.pose.orientation.w = st.q.w(); odom.pose.pose.orientation.x = st.q.x(); odom.pose.pose.orientation.y = st.q.y(); odom.pose.pose.orientation.z = st.q.z();
    odom.twist.twist.linear.x = st.v.x(); odom.twist.twist.linear.y = st.v.y(); odom.twist.twist.linear.z = st.v.z();
    odom.twist.twist.angular.x = gyro.x() - st.bg.x(); // bias-corrected
    odom.twist.twist.angular.y = gyro.y() - st.bg.y();
    odom.twist.twist.angular.z = gyro.z() - st.bg.z();
    // covariance: fill position 3x3 from P[6:9,6:9]
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) odom.pose.covariance[r*6 + c] = st.P(6+r, 6+c);
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) odom.twist.covariance[r*6 + c] = st.P(3+r, 3+c);
    pub_odom_->publish(odom);

    if (imu_count_ % 1000 == 0) {
      RCLCPP_INFO(get_logger(), "imu %ld dt %.4f v [%.2f %.2f %.2f] p [%.2f %.2f %.2f]",
        imu_count_, dt, st.v.x(), st.v.y(), st.v.z(), st.p.x(), st.p.y(), st.p.z());
    }
  }

  void watchdog() {
    auto t = now();
    double a_imu = has_last_imu_ ? (t - last_imu_wall_time_).seconds() : 1e9;
    double a_j = has_qj_ ? (t - last_joints_time_).seconds() : 1e9;
    double a_c = has_contacts_ ? (t - last_contacts_time_).seconds() : 1e9;
    if (a_imu > 0.5 || a_j > 0.5 || a_c > 0.5) {
      RCLCPP_WARN(get_logger(), "watchdog: stale imu %.2fs joints %.2fs contacts %.2fs", a_imu, a_j, a_c);
    }
  }

  otolith::FusionEKF ekf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_contacts_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_watchdog_;

  std::mutex mtx_;
  std::array<double,12> latest_qj_{}; bool has_qj_=false; rclcpp::Time last_joints_time_;
  std::array<uint8_t,4> latest_contacts_{}; bool has_contacts_=false; rclcpp::Time last_contacts_time_;
  bool has_last_imu_=false; double last_imu_header_time_=0; rclcpp::Time last_imu_wall_time_; bool initialized_=false;
  long imu_count_=0;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
