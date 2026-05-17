#include "rclcpp/rclcpp.hpp"
#include "asr_logger/logger_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<asr_logger::LoggerNode>());
    rclcpp::shutdown();
    return 0;
}
