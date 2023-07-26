#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "festival_ur_core/festival.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<FestivalNode>());
  rclcpp::shutdown();
  return 0;
}
