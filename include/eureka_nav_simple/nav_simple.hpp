#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>

struct Arrow {
public:
  Arrow() = default;
  Arrow(std::string& dir, double& range, double& angle, double& certainty)
  {
    direction = dir;
    range     = range;
    angle     = angle;
    certainty = certainty;
  }
public:
  std::string direction;
  double range;
  double angle;
  double certainty;
};

class NavSimple : public rclcpp::Node {
public:
  NavSimple();
  ~NavSimple();

private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void callback_2(const sensor_msgs::msg::JointState::SharedPtr msg);
  auto arrowFilter(double range_min, double range_max, double angle_max, double certainty_min);
  bool findArrow();
  void approachArrow();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_arrow;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_commands;
  rclcpp::TimerBase::SharedPtr pipeline;

  std::vector<Arrow> arrow_vector;

  int lost_counter;

  double threshold_range;
  bool autonomous_mode;
  double p_gain;
  double maximum_range;
};