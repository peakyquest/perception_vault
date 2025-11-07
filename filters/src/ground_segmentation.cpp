#include "rclcpp/rclcpp.hpp"

class GroundSegmentation : public rclcpp::Node
{
public:
  GroundSegmentation()
  : Node("ground_segmentation")
  {
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "ground_segmentation");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmentation>());
  rclcpp::shutdown();
  return 0;
}
