#include "lidar.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

LidarNode::LidarNode() : Node("lidar_node"), file_i2c(-1) {
    publisher_ = create_publisher<sensor_msgs::msg::Range>("lidar/distance", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&LidarNode::read_and_publish, this));
    
    if (i2c_init() < 0 || i2c_connect(LIDAR_ADDR_DEFAULT) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize I2C");
        rclcpp::shutdown();
        return;
    }
    
    configure(0, LIDAR_ADDR_DEFAULT);
    RCLCPP_INFO(get_logger(), "Lidar initialized successfully");
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
    write(file_i2c, &reg_addr, 1);
    return read(file_i2c, data, num_bytes);
}

// Moving average filter for distance
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
        auto msg = sensor_msgs::msg::Range();
        msg.header.stamp = now();
        msg.header.frame_id = "lidar_frame";
        msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        msg.field_of_view = 0.1;
        msg.min_range = 0.05;    // 5 cm
        msg.max_range = 40.0;    // 40 meters
        msg.range = distance / 100.0; // Convert cm to meters
        publisher_->publish(msg);
        sample_count = 0; // Reset counter
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}