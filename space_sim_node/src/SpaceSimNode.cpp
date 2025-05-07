// SpaceSimNode.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>

#include "SpaceSim.hpp"

using std::placeholders::_1;

class SpaceSimNode : public rclcpp::Node {
public:
  SpaceSimNode() : Node("spacesim_node") {
    // Publishers
    crs_pose_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/crs/state/pose", 10);
    crs_twist_pub_  = create_publisher<geometry_msgs::msg::TwistStamped>("/crs/state/twist", 10);
    tgt_pose_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/tgt/state/pose", 10);
    tgt_twist_pub_  = create_publisher<geometry_msgs::msg::TwistStamped>("/tgt/state/twist", 10);
    crs_joint_pub_  = create_publisher<sensor_msgs::msg::JointState>("/crs/state/joint_states", 10);
    crs_pose_js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/crs/state/pose_joint", 10);
    tgt_pose_js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/tgt/state/pose_joint", 10);
    gripper_q_pub_   = create_publisher<sensor_msgs::msg::JointState>("/crs/state/gripper_q", 10);

    // Subscribers
    crs_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/crs/command/wrench", 10, std::bind(&SpaceSimNode::crsWrenchCb, this, _1));
    crs_tau_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/crs/command/torque", 10, std::bind(&SpaceSimNode::crsTauCb, this, _1));
    crs_rw_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/crs/command/RW_torque", 10, std::bind(&SpaceSimNode::crsRWcb, this, _1));
    tgt_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/tgt/command/wrench", 10, std::bind(&SpaceSimNode::tgtWrenchCb, this, _1));
    tgt_rw_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/tgt/command/RW_torque", 10, std::bind(&SpaceSimNode::tgtRWcb, this, _1));
    gripper_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/crs/GripperState", 10, std::bind(&SpaceSimNode::gripperCb, this, _1));

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&SpaceSimNode::updateLoop, this));
    sim_.initialize(q_init, CRS_T, TGT_T);

  }

private:
  void crsWrenchCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    crs_f_.head<3>() = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    crs_f_.tail<3>() = Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  }
  void crsTauCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->effort.size() == 7)
      for (int i = 0; i < 7; ++i) crs_tau_[i] = msg->effort[i];
  }
  void crsRWcb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->effort.size() == 6)
      for (int i = 0; i < 6; ++i) crs_rw_[i] = msg->effort[i];
  }
  void tgtWrenchCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    tgt_f_.head<3>() = Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    tgt_f_.tail<3>() = Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  }
  void tgtRWcb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->effort.size() == 6)
      for (int i = 0; i < 6; ++i) tgt_rw_[i] = msg->effort[i];
  }
  void gripperCb(const std_msgs::msg::Int32::SharedPtr msg) {
    gripper_state_ = msg->data;
  }

  void updateLoop() {
    sim_.setInput(crs_f_, crs_tau_, tgt_f_, crs_rw_, tgt_rw_, gripper_state_);

    // publish pose, twist, joint states
    publishPose(sim_.CRS_SatPos, crs_pose_pub_);
    publishPose(sim_.TGT_SatPos, tgt_pose_pub_);
    publishTwist(sim_.CRS_SatVel, crs_twist_pub_);
    publishTwist(sim_.TGT_SatVel, tgt_twist_pub_);
    publishJoint(sim_.q, sim_.qdot, sim_.gripper_q,crs_joint_pub_);
    publishPoseJoint(sim_.CRS_SatPos, crs_pose_js_pub_);
    publishPoseJoint(sim_.TGT_SatPos, tgt_pose_js_pub_);
    publishGripper(sim_.gripper_q, gripper_q_pub_);
  }

  void publishPose(const Eigen::VectorXd& pose7, const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.pose.position.x = pose7(0);
    msg.pose.position.y = pose7(1);
    msg.pose.position.z = pose7(2);
    msg.pose.orientation.x = pose7(4);
    msg.pose.orientation.y = pose7(5);
    msg.pose.orientation.z = pose7(6);
    msg.pose.orientation.w = pose7(3);
    pub->publish(msg);
  }

  void publishTwist(const Eigen::VectorXd& vel6, const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& pub) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.twist.linear.x  = vel6(0);
    msg.twist.linear.y  = vel6(1);
    msg.twist.linear.z  = vel6(2);
    msg.twist.angular.x = vel6(3);
    msg.twist.angular.y = vel6(4);
    msg.twist.angular.z = vel6(5);
    pub->publish(msg);
  }

  void publishJoint(const Eigen::VectorXd& q,
    const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& gripper_q,        // size == 2
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
{
sensor_msgs::msg::JointState msg;
msg.header.stamp = now();

static const std::array<std::string, 9> names = {
"joint1","joint2","joint3","joint4","joint5","joint6","joint7",
"gripper_r_prismatic","gripper_l_prismatic"};
msg.name.assign(names.begin(), names.end());

/* ── position ─────────────────────────────────────────── */
msg.position.resize(9);

// 0~6 : 관절 각도, ±π wrap
std::transform(q.data(), q.data() + 7, msg.position.begin(),
   [](double a){
     constexpr double TWO_PI = 2.0 * M_PI;
     a = std::fmod(a + M_PI, TWO_PI);   // [0, 2π)
     if (a < 0) a += TWO_PI;
     return a - M_PI;                   // (-π, π]
   });

// 7, 8 : 그리퍼 프리즘 조인트
msg.position[7] = gripper_q(0);
msg.position[8] = gripper_q(1);

/* ── velocity ─────────────────────────────────────────── */
msg.velocity.assign(9, 0.0);            // 전체 0 으로 초기화
std::copy(qdot.data(), qdot.data() + 7, // 앞 7개만 복사
msg.velocity.begin());

/* ── effort (빈값) ────────────────────────────────────── */
msg.effort.assign(9, 0.0);

pub->publish(msg);
}


  void publishPoseJoint(const Eigen::VectorXd& pose7, const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    static const std::array<std::string, 7> names = {"qw","qx","qy","qz","x","y","z"};
    msg.name.assign(names.begin(), names.end());
    msg.position.resize(7);
    msg.position[0] = pose7(0);
    msg.position[1] = pose7(1);
    msg.position[2] = pose7(2);
    msg.position[3] = pose7(3);
    msg.position[4] = pose7(4);
    msg.position[5] = pose7(5);
    msg.position[6] = pose7(6);
    pub->publish(msg);
  }

  void publishGripper(const Eigen::Vector2d& gq, const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {"gripper_l", "gripper_r"};
    msg.position = {gq(0), gq(1)};
    msg.velocity = {0.0, 0.0};
    msg.effort   = {0.0, 0.0};
    pub->publish(msg);
  }

  rclcpp::Time now() const { return const_cast<rclcpp::Clock*>(this->get_clock().get())->now(); }


  // Members
  SpaceSim sim_;
  Eigen::VectorXd crs_f_  = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd crs_tau_ = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd crs_rw_  = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tgt_f_  = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tgt_rw_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(7);
Eigen::Matrix4d CRS_T = Eigen::Matrix4d::Identity();
Eigen::Matrix4d TGT_T = Eigen::Matrix4d::Identity();

  int gripper_state_ = 0;

  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr crs_wrench_sub_, tgt_wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr crs_tau_sub_, crs_rw_sub_, tgt_rw_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gripper_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr crs_pose_pub_, tgt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr crs_twist_pub_, tgt_twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr crs_joint_pub_, crs_pose_js_pub_, tgt_pose_js_pub_, gripper_q_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpaceSimNode>());
  rclcpp::shutdown();
  return 0;
}

