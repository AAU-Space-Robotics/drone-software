#include <rclcpp/rclcpp.hpp>
#include "fci_library.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightControllerInterface>());
    rclcpp::shutdown();
    return 0;
}
