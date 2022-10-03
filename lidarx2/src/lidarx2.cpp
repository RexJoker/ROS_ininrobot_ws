#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "x2device.h"
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    using namespace std::chrono_literals;
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  x2device device = { 0 };
  x2data data = { 0 };
  int ret = init_device(&device);
  while (rcv_data(&device, &data) == 0) {
        if (data.checkcode == CHECK_SUM) {
            for (int i = 0; i < data.length; i++) {
                if (data.distance[i] != 0) {
                    //printf("angle: %lf    dis:%lf\n", data.angles[i] / 64.0, data.distance[i] / 4.0);
                    std::cout << "angle: " << (data.angles[i]/64.0) << " dis: " << (data.distance[i]/4.0) << std::endl;
                }
            }
        }
  }
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}