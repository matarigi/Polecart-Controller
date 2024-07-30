#include "IPolecart_Controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>
using std::placeholders::_1;

/**
 * Polecart_Controller is a ROS2 node and has a subscibre to an input topic where it reads the angle value,
 * ask to a PID the output for the car_pole and publish in the topic
 */
class Polecart_Controller : public rclcpp::Node, IPolecart_Controller
{
  public:
    Polecart_Controller(std::string node_name, std::string input_topic, std::string output_topic)
      : Node(node_name)
    {
      this->init(input_topic, output_topic);
      this->start();
    }

    void init(std::string input_topic, std::string output_topic) {
      this->input_topic_ = input_topic;
      this->output_topic_ = output_topic;
    }

    // Initialize the subscriber and publisher
    void start() {
      this->subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        this->input_topic_ , 10, std::bind(&Polecart_Controller::read_angle_message, this, _1));
      this->publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>(this->output_topic_, 10);
    }

  private:
    // Topics
    std::string input_topic_;
    std::string output_topic_;

    // Subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;

    // Subscriber callback
    void read_angle_message(const sensor_msgs::msg::JointState msg) const
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.position[0] << "'");
      this -> publish_force_message(msg.position[0]);
    }

    // Publisher function
    void publish_force_message(_Float64 force) const
    {
      auto message = geometry_msgs::msg::Wrench();
      message.force.y = force;
      publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  const std::string input_topic = "/cart/pole_state";
  const std::string output_topic = "/cart/force";
  const std::string node_name = "Polecart_Controller";

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Polecart_Controller>(node_name, input_topic, output_topic);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}