#ifndef LIDAR_H
#define LIDAR_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <array>

class LidarNode : public rclcpp::Node {
public:
    LidarNode();

private:
    static constexpr uint8_t LIDAR_ADDR_DEFAULT = 0x62;
    static constexpr size_t SAMPLES_PERIOD = 35; // 350 Hz * 0.1 s = 35 samples
    int file_i2c;
    rclcpp::Publisher<px4_msgs::msg::DistanceSensor>::SharedPtr px4_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<double, SAMPLES_PERIOD> moving_average_values = {0};

    int32_t i2c_init();
    int32_t i2c_connect(uint8_t address);
    void configure(uint8_t configuration, uint8_t address);
    void take_range(uint8_t address);
    uint8_t get_busy_flag(uint8_t address);
    void wait_for_busy(uint8_t address);
    uint16_t read_distance(uint8_t address);
    int32_t i2c_write(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address);
    int32_t i2c_read(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address);
    double moving_average(float new_value);
    void read_and_publish();

    // Register addresses for LIDAR-Lite v3HP
    static constexpr uint8_t LL_ACQ_CMD = 0x00;
    static constexpr uint8_t LL_STATUS = 0x01;
    static constexpr uint8_t LL_SIG_CNT_VAL = 0x02;
    static constexpr uint8_t LL_ACQ_CONFIG = 0x04;
    static constexpr uint8_t LL_REF_CNT_VAL = 0x12;
    static constexpr uint8_t LL_THRESH_BYPASS = 0x1c;
    static constexpr uint8_t LL_DISTANCE = 0x8f;
};

#endif // LIDAR_H