#ifndef LIDAR_NODE_H
#define LIDAR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <cstdint>

#define LIDAR_ADDR_DEFAULT 0x62
#define LL_ACQ_CMD       0x00
#define LL_STATUS        0x01
#define LL_SIG_CNT_VAL   0x02
#define LL_ACQ_CONFIG    0x04
#define LL_DISTANCE      0x0f
#define LL_REF_CNT_VAL   0x12
#define LL_THRESH_BYPASS 0x1c

class LidarNode : public rclcpp::Node {
public:
    LidarNode();

private:
    int32_t file_i2c;
    rclcpp::Publisher<px4_msgs::msg::DistanceSensor>::SharedPtr px4_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int32_t i2c_init();
    int32_t i2c_connect(uint8_t address);
    void configure(uint8_t configuration, uint8_t address);
    void take_range(uint8_t address);
    uint8_t get_busy_flag(uint8_t address);
    void wait_for_busy(uint8_t address);
    uint16_t read_distance(uint8_t address);
    int32_t i2c_write(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address);
    int32_t i2c_read(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address);
    void read_and_publish();
    double moving_average(double new_value);
    std::array<double, 5> moving_average_values = {0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif