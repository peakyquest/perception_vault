#include "rclcpp/rclcpp.hpp"

class OutliersFilters : public rclcpp::Node
{
public:
  OutliersFilters()
  : Node("outliers_filters")
  {
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "outliers_filters");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutliersFilters>());
  rclcpp::shutdown();
  return 0;
}
