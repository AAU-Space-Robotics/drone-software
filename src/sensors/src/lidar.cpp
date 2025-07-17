#include "lidar.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

LidarNode::LidarNode() : Node("lidar_node"), file_i2c(-1) {
    px4_publisher_ = create_publisher<px4_msgs::msg::DistanceSensor>("/fmu/in/distance_sensor", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&LidarNode::read_and_publish, this));
    
    if (i2c_init() < 0 || i2c_connect(LIDAR_ADDR_DEFAULT) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize I2C");
        rclcpp::shutdown();
        return;
    }
    
    configure(0, LIDAR_ADDR_DEFAULT);
    RCLCPP_INFO(get_logger(), "Garmin LIDAR-Lite v3HP initialized successfully");
}

int32_t LidarNode::i2c_init() {
    const char* filename = "/dev/i2c-1";
    file_i2c = open(filename, O_RDWR);
    if (file_i2c < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open I2C bus");
        return -1;
    }
    return 0;
}

int32_t LidarNode::i2c_connect(uint8_t address) {
    if (ioctl(file_i2c, I2C_SLAVE, address) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to I2C device");
        return -1;
    }
    return 0;
}

void LidarNode::configure(uint8_t configuration, uint8_t address) {
    uint8_t sigCountMax = 0x80, acqConfigReg = 0x08, refCountMax = 0x05, thresholdBypass = 0x00;

    switch (configuration) {
        case 1: // Short range, high speed
            sigCountMax = 0x1d;
            refCountMax = 0x03;
            break;
        case 2: // Default range, higher speed short range
            acqConfigReg = 0x00;
            refCountMax = 0x03;
            break;
        case 3: // Maximum range
            sigCountMax = 0xff;
            break;
        case 4: // High sensitivity
            thresholdBypass = 0x80;
            break;
        case 5: // Low sensitivity
            thresholdBypass = 0xb0;
            break;
        case 6: // Short range, high speed, higher error
            sigCountMax = 0x04;
            acqConfigReg = 0x01;
            refCountMax = 0x03;
            break;
        default: // Balanced performance
            break;
    }

    i2c_write(LL_SIG_CNT_VAL, &sigCountMax, 1, address);
    i2c_write(LL_ACQ_CONFIG, &acqConfigReg, 1, address);
    i2c_write(LL_REF_CNT_VAL, &refCountMax, 1, address);
    i2c_write(LL_THRESH_BYPASS, &thresholdBypass, 1, address);
}

void LidarNode::take_range(uint8_t address) {
    uint8_t command = 0x04;
    i2c_write(LL_ACQ_CMD, &command, 1, address);
}

uint8_t LidarNode::get_busy_flag(uint8_t address) {
    uint8_t status = 0;
    i2c_read(LL_STATUS, &status, 1, address);
    return status & 0x01;
}

void LidarNode::wait_for_busy(uint8_t address) {
    while (get_busy_flag(address)) {}
}

uint16_t LidarNode::read_distance(uint8_t address) {
    uint8_t dist_bytes[2] = {0};
    i2c_read(LL_DISTANCE | 0x80, dist_bytes, 2, address);
    return (dist_bytes[0] << 8) | dist_bytes[1];
}

int32_t LidarNode::i2c_write(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address) {
    uint8_t buffer[2];
    int32_t result = 0;
    i2c_connect(address);
    for (uint8_t i = 0; i < num_bytes; ++i) {
        buffer[0] = reg_addr + i;
        buffer[1] = data[i];
        result |= write(file_i2c, buffer, 2);
    }
    return result;
}

int32_t LidarNode::i2c_read(uint8_t reg_addr, uint8_t* data, uint8_t num_bytes, uint8_t address) {
    i2c_connect(address);
    if (write(file_i2c, &reg_addr, 1) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to write register address");
        return -1;
    }
    return read(file_i2c, data, num_bytes);
}

double LidarNode::moving_average(double new_value) {
    static std::array<double, 5> values = {0};
    static size_t index = 0;
    static size_t count = 0;

    values[index] = new_value;
    index = (index + 1) % values.size();
    if (count < values.size()) ++count;

    double sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        sum += values[i];
    }
    return sum / count;
}

void LidarNode::read_and_publish() {
    static int sample_count = 0; // Counter for downsampling

    take_range(LIDAR_ADDR_DEFAULT);
    wait_for_busy(LIDAR_ADDR_DEFAULT);
    auto distance = read_distance(LIDAR_ADDR_DEFAULT);
    distance = moving_average(static_cast<double>(distance)); // Apply moving average filter

    // Downsample: Publish every 4th sample (200 Hz to 50 Hz)
    if (++sample_count >= 4) {
        // Only publish valid distances (5 cm to 40 m)
        if (distance >= 5 && distance <= 4000) {
            auto px4_msg = px4_msgs::msg::DistanceSensor();
            px4_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Microseconds
            px4_msg.device_id = 1234; // Unique ID for the sensor
            px4_msg.min_distance = 0.05; // 5 cm
            px4_msg.max_distance = 40.0; // 40 meters
            px4_msg.current_distance = distance / 100.0; // Convert cm to meters
            px4_msg.variance = (distance / 100.0 < 2.0) ? 0.01 : 0.0025; // ±5 cm for <2 m, ±2.5 cm for >2 m
            px4_msg.signal_quality = -1; // Unknown quality
            px4_msg.type = 0; // MAV_DISTANCE_SENSOR_LASER
            px4_msg.h_fov = 0.0; // 1D LiDAR has no FOV
            px4_msg.v_fov = 0.0;
            px4_msg.q = {0.0, 0.0, 0.0, 1.0}; // Identity quaternion for downward-facing
            px4_msg.orientation = 25; // ROTATION_DOWNWARD_FACING
            px4_publisher_->publish(px4_msg);
        } else {
            RCLCPP_WARN(get_logger(), "Invalid distance reading: %.2d cm", distance);
        }
        sample_count = 0; // Reset counter
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}