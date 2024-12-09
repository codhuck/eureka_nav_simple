#include "eureka_nav_simple/nav_simple.hpp"

using namespace std::chrono_literals;

NavSimple::NavSimple()
    : Node("nav_simple"), threshold_range(1.5), autonomous_mode(0), p_gain(0.05), maximum_range(10.0) {
    publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_arrow = create_subscription<sensor_msgs::msg::JointState>(
        "arrow_detection", 10, std::bind(&NavSimple::callback, this, std::placeholders::_1));
    subscription_commands = create_subscription<sensor_msgs::msg::JointState>(
        "autonomous_commands", 10, std::bind(&NavSimple::callback_2, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NavSimple started!");
    std::thread(&NavSimple::pipeline, this).detach();
}

NavSimple::~NavSimple() {
    RCLCPP_INFO(get_logger(), "NavSimple stopped!");
}

void NavSimple::callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    arrow_direction = msg->name;
    arrow_range = msg->position;
    arrow_angle = msg->velocity;
    arrow_certainty = msg->effort;
}

void NavSimple::callback_2(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (auto it = std::find(msg->name.begin(), msg->name.end(), "autonomous_mode"); it != msg->name.end()) {
        autonomous_mode = msg->position[std::distance(msg->name.begin(), it)];
    }
}

std::optional<std::tuple<double, double, double>> NavSimple::arrowFilter(
    double range_min, double range_max, double angle_max, double certainty_min) {
    if (!arrow_range.empty()) {
        for (size_t i = 0; i < arrow_range.size(); ++i) {
            if (arrow_range[i] > range_min && arrow_range[i] < range_max &&
                std::abs(arrow_angle[i]) < angle_max && arrow_certainty[i] > certainty_min) {
                return std::make_tuple(arrow_range[i], arrow_angle[i], arrow_certainty[i]);
            }
        }
    }
    return std::nullopt;
}

void NavSimple::findArrow() {
    geometry_msgs::msg::Twist message;
    int direction = 1;

    for (size_t i = 0; i < arrow_range.size(); ++i) {
        if (arrow_range[i] < 2.0 && arrow_direction[i] == "right" && arrow_certainty[i] > 0.5) {
            RCLCPP_INFO(get_logger(), "Right arrow!");
            direction = -1;
            break;
        }
    }

    message.angular.z = 100.0;
    message.linear.x = direction * 0.05;
    publisher->publish(message);

    while (arrowFilter(2.0, 10.0, 15.0, 0.6) == std::nullopt && autonomous_mode == 1) {
        publisher->publish(message);
        std::this_thread::sleep_for(100ms);
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
}

void NavSimple::approachArrow() {
    geometry_msgs::msg::Twist message;
    message.linear.x = 0.07;

    int arrival = 0, lost_counter = 0;
    double error = 0.0;

    while (arrival < 1 && autonomous_mode == 1) {
        auto arrow = arrowFilter(1.0, 1.5, 100.0, 0.6);
        if (arrow) {
            RCLCPP_INFO(get_logger(), "Arrived!");
            std::this_thread::sleep_for(5s);
            arrival = 1;
            break;
        }

        arrow = arrowFilter(1.5, maximum_range, 100.0, 0.6);
        if (arrow) {
            lost_counter = 0;
            maximum_range = std::get<0>(*arrow) + 0.5;
            error = std::get<1>(*arrow);
        } else if (++lost_counter > 10) {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher->publish(message);
            RCLCPP_WARN(get_logger(), "Arrow lost!");
            return;
        }

        message.angular.z = -error * p_gain;
        publisher->publish(message);
        std::this_thread::sleep_for(100ms);
    }

    message.linear.x = 0.0;
    message.angular.z = 0.0;
    publisher->publish(message);
}

void NavSimple::pipeline() {
    while (rclcpp::ok()) {
        if (autonomous_mode == 1) {
            findArrow();
            std::this_thread::sleep_for(2s);
            approachArrow();
        }
        std::this_thread::sleep_for(5s);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavSimple>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}