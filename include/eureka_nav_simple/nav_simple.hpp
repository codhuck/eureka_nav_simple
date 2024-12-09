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
    std::string direction;
    double range;
    double angle;
    double certainty;
    int repetition;
};

class NavSimple : public rclcpp::Node {
public:
    NavSimple();
    ~NavSimple();

private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void callback_2(const sensor_msgs::msg::JointState::SharedPtr msg);
    std::optional<std::tuple<double, double, double>> arrowFilter(double range_min, double range_max, double angle_max, double certainty_min);
    void findArrow();
    void approachArrow();
    void pipeline();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_arrow;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_commands;

    std::vector<std::string> arrow_direction;
    std::vector<double> arrow_range;
    std::vector<double> arrow_angle;
    std::vector<double> arrow_certainty;

    double threshold_range;
    int autonomous_mode;
    double p_gain;
    double maximum_range;
};