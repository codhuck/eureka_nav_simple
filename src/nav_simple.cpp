#include "eureka_nav_simple/nav_simple.hpp"

using namespace std::chrono_literals;

NavSimple::NavSimple()
  : Node("nav_simple_cv"),
    threshold_range(1.5),
    autonomous_mode(false)
{
  publisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  subscription_arrow = create_subscription<sensor_msgs::msg::JointState>(
      "arrow_detection", 10, std::bind(&NavSimple::callback, this, std::placeholders::_1));
  subscription_commands = create_subscription<sensor_msgs::msg::JointState>(
      "autonomous_commands", 10, std::bind(&NavSimple::callback_2, this, std::placeholders::_1));

  p_gain = declare_parameter("pid_gain", 0.05);
  maximum_range = declare_parameter("maximum_range", 10.0);

  pipeline = create_wall_timer(5s, [this]() {
              if (autonomous_mode && findArrow())
              {
                approachArrow();
              }});

  RCLCPP_INFO(get_logger(), "NavSimple started!");
}

NavSimple::~NavSimple()
{
  RCLCPP_INFO(get_logger(), "NavSimple stopped!");
}

void NavSimple::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  size_t msg_size = msg->name.size();

  arrow_vector.clear();
  arrow_vector.reserve(msg_size);

  auto pos = arrow_vector.begin();

  for(size_t i = 0; i < msg_size; ++i)
  {
    pos = arrow_vector.emplace(pos, msg->name[i], msg->position[i], msg->velocity[i], msg->effort[i]);
  }
}

void NavSimple::callback_2(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (auto it = std::find(msg->name.begin(), msg->name.end(), "autonomous_mode"); it != msg->name.end())
    autonomous_mode = msg->position[std::distance(msg->name.begin(), it)];
}


auto NavSimple::arrowFilter(double range_min, double range_max, double angle_max, double certainty_min)
{
  return std::find_if(arrow_vector.begin(), arrow_vector.end(),[range_min, range_max, angle_max, certainty_min](Arrow& obj)
                                                                                       { return obj.range > range_min &&
                                                                                                obj.range < range_max &&
                                                                                                std::abs(obj.angle) < angle_max &&
                                                                                                obj.certainty > certainty_min;});
}

bool NavSimple::findArrow()
{
  int direction;

  bool cond = std::any_of(arrow_vector.begin(), arrow_vector.end(),[](Arrow& obj){ return obj.range < 2.0 &&
                                                                                          obj.direction == "right" &&
                                                                                          obj.certainty > 0.5;});

  cond ? direction = 1
       : direction = -1;

  if(arrowFilter(2.0, 10.0, 15.0, 0.6) == arrow_vector.end())
  {
    geometry_msgs::msg::Twist message;
    message.angular.z = 100.0;
    message.linear.x = direction * 0.05;
    publisher->publish(message);
    return false;
  }
  else
    return true;
}

void NavSimple::approachArrow()
{
  geometry_msgs::msg::Twist message;
  double error;

  message.linear.x = 0.07;

  if(arrowFilter(1.0, 1.5, 100.0, 0.6) != arrow_vector.end())
  {
    RCLCPP_INFO(get_logger(), "eureka arrived");
  }
  else
  {
    auto arrow = arrowFilter(1.5, maximum_range, 100.0, 0.6);

    if(arrow != arrow_vector.end())
    {
      lost_counter = 0;
      maximum_range = (*arrow).range + 0.5;
      error = (*arrow).angle;
      message.angular.z = -error * p_gain;
      publisher->publish(message);
    }
    else if (lost_counter > 10)
    {
      lost_counter++;
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      publisher->publish(message);
      RCLCPP_WARN(get_logger(), "Arrow lost!");
    }
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavSimple>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}