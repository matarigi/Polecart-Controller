#include "IPolecart_Controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "PID_Controller.hpp"
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
    Polecart_Controller(std::string node_name, std::string input_topic, std::string output_topic, double delta_time)
      : Node(node_name)
    {
      this -> delta_time = delta_time;
      this->init(input_topic, output_topic);
      this->start();
    }

    void init(std::string input_topic, std::string output_topic) 
    {
      this->input_topic_ = input_topic;
      this->output_topic_ = output_topic;

      this -> k_pid.k_derivate = 20;
      this -> k_pid.k_integral = 15;
      this -> k_pid.k_proportional = 35;

      controller.init(this -> k_pid);

      controller.set_set_point(0.0);
    }

    // Initialize the subscriber and publisher
    void start() 
    {
      this->subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        this->input_topic_ , 10, std::bind(&Polecart_Controller::subscriber_callback, this, _1));
      this->publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>(this->output_topic_, 10);
    }

  private:
    // Topics
    std::string input_topic_;
    std::string output_topic_;

    // Subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;

    // PID Controller and its constants
    PID_Controller controller;
    PID_Constants k_pid;

    double delta_time;

    void subscriber_callback(const sensor_msgs::msg::JointState msg)
    {
      double force = this -> controller.output_pid_calculation(msg.position[0], this -> delta_time);
      this -> publish_force_message(force);
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
  const double delta_time = 0.05;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Polecart_Controller>(node_name, input_topic, output_topic, delta_time);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}