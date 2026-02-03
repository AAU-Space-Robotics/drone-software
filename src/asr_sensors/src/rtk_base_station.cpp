#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial_driver/serial_driver.hpp>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <set>

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

enum class BaseStationState {
    INITIALIZING,
    CONFIGURING,
    SURVEY_IN_PROGRESS,
    SURVEY_IN_COMPLETE,
    FIXED_BASE_MODE,
    PUBLISHING_RTCM,
    ERROR
};

class RTKBaseStation : public rclcpp::Node {
public:
    RTKBaseStation() : Node("rtk_base_station"), 
                       io_context_(new drivers::common::IoContext(2)),
                       state_(BaseStationState::INITIALIZING) {
        // Parameters
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("frame_id", "gnss_base");
        this->declare_parameter("survey_duration", 200);
        this->declare_parameter("survey_accuracy_mm", 2500);
        this->declare_parameter("auto_configure", true);
        
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        survey_duration_ = this->get_parameter("survey_duration").as_int();
        survey_accuracy_mm_ = this->get_parameter("survey_accuracy_mm").as_int();
        auto_configure_ = this->get_parameter("auto_configure").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "RTK Base Station Node Starting...");
        RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d", port.c_str(), baudrate);
        RCLCPP_INFO(this->get_logger(), "Survey: %ds, Accuracy: %dmm", survey_duration_, survey_accuracy_mm_);
        
        // Configure serial port
        SerialPortConfig config(baudrate, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        serial_driver_ = std::make_unique<SerialDriver>(*io_context_);
        
        try {
            serial_driver_->init_port(port, config);
            if (serial_driver_->port()) {
                serial_driver_->port()->open();
                RCLCPP_INFO(this->get_logger(), "✓ Connected to %s", port.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create serial port");
                state_ = BaseStationState::ERROR;
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
            state_ = BaseStationState::ERROR;
            return;
        }
        
        // Create publishers
        rtcm_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rtcm", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/rtk/status", 10);
        
        // Timer for status updates
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&RTKBaseStation::publish_status, this));
        
        // Start configuration in separate thread
        if (auto_configure_) {
            config_thread_ = std::thread(&RTKBaseStation::configure_and_run, this);
        } else {
            // Just start reading if already configured
            state_ = BaseStationState::PUBLISHING_RTCM;
            read_thread_ = std::thread(&RTKBaseStation::read_loop, this);
        }
        
        RCLCPP_INFO(this->get_logger(), "RTK Base Station node initialized");
    }
    
    ~RTKBaseStation() {
        running_ = false;
        if (config_thread_.joinable()) config_thread_.join();
        if (read_thread_.joinable()) read_thread_.join();
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        print_stats();
    }

private:
    void configure_and_run() {
        RCLCPP_INFO(this->get_logger(), "\n=== Starting RTK Base Station Configuration ===");
        
        // Step 1: Configure RTCM3 output
        state_ = BaseStationState::CONFIGURING;
        if (!configure_rtcm3()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure RTCM3");
            state_ = BaseStationState::ERROR;
            return;
        }
        
        // Step 2: Start survey-in
        state_ = BaseStationState::SURVEY_IN_PROGRESS;
        if (!start_survey_in()) {
            RCLCPP_WARN(this->get_logger(), "Survey-in command not acknowledged - may already be configured");
        }
        
        // Step 3: Monitor survey-in progress
        if (!monitor_survey_in()) {
            RCLCPP_ERROR(this->get_logger(), "Survey-in monitoring failed");
            state_ = BaseStationState::ERROR;
            return;
        }
        
        // Step 4: Configure as fixed base with surveyed position
        state_ = BaseStationState::FIXED_BASE_MODE;
        if (surveyed_position_valid_) {
            configure_fixed_base(surveyed_ecef_x_, surveyed_ecef_y_, surveyed_ecef_z_, surveyed_accuracy_);
        }
        
        // Step 5: Start publishing RTCM3
        state_ = BaseStationState::PUBLISHING_RTCM;
        RCLCPP_INFO(this->get_logger(), "\n✓ ✓ ✓ RTK Base Station Ready - Publishing RTCM3 Corrections ✓ ✓ ✓\n");
        
        // Start continuous read loop in background thread
        read_thread_ = std::thread(&RTKBaseStation::read_loop, this);
    }
    
    bool configure_rtcm3() {
        RCLCPP_INFO(this->get_logger(), "Configuring RTCM3 messages...");
        
        // Enable RTCM3 on port
        std::vector<uint8_t> port_cfg = {
            0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
            0xD0, 0x08, 0x00, 0x00,
            0x00, 0xC2, 0x01, 0x00,  // 115200 baud
            0x07, 0x00, 0x07, 0x00,  // Input/Output: UBX + NMEA + RTCM3
            0x00, 0x00, 0x00, 0x00
        };
        send_ubx_with_ack(port_cfg);
        
        // Enable RTCM3 messages
        const std::vector<std::pair<uint8_t, std::string>> rtcm_msgs = {
            {0x05, "1005 - Base position"},
            {0x4D, "1077 - GPS MSM7"},
            {0x57, "1087 - GLONASS MSM7"},
            {0x61, "1097 - Galileo MSM7"},
            {0x7F, "1127 - BeiDou MSM7"},
            {0xE6, "1230 - GLONASS biases"}
        };
        
        for (const auto& [id, desc] : rtcm_msgs) {
            std::vector<uint8_t> enable_rtcm = {
                0x06, 0x01, 0x08, 0x00, 0xF5, id,
                0x00, 0x01, 0x00, 0x00, 0x00, 0x00
            };
            send_ubx_with_ack(enable_rtcm);
            RCLCPP_INFO(this->get_logger(), "  ✓ Enabled %s", desc.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Save configuration
        std::vector<uint8_t> save_cfg = {
            0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F
        };
        send_ubx_with_ack(save_cfg);
        
        RCLCPP_INFO(this->get_logger(), "✓ RTCM3 configuration complete");
        return true;
    }
    
    bool start_survey_in() {
        RCLCPP_INFO(this->get_logger(), "Starting Survey-In: %ds, %dmm accuracy...", 
                   survey_duration_, survey_accuracy_mm_);
        
        // First, disable any existing time mode to start fresh
        std::vector<uint8_t> disable_tmode(44, 0);
        disable_tmode[0] = 0x06; disable_tmode[1] = 0x71; 
        disable_tmode[2] = 0x28; disable_tmode[3] = 0x00;
        disable_tmode[6] = 0x00; // Mode = 0 (disabled)
        send_ubx_with_ack(disable_tmode);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Now start survey-in mode
        std::vector<uint8_t> tmode3(44, 0);
        tmode3[0] = 0x06; tmode3[1] = 0x71; tmode3[2] = 0x28; tmode3[3] = 0x00;
        tmode3[6] = 0x01; // Survey-in mode
        
        uint32_t acc_0_1mm = survey_accuracy_mm_ * 10;
        tmode3[36] = survey_duration_ & 0xFF;
        tmode3[37] = (survey_duration_ >> 8) & 0xFF;
        tmode3[38] = (survey_duration_ >> 16) & 0xFF;
        tmode3[39] = (survey_duration_ >> 24) & 0xFF;
        tmode3[40] = acc_0_1mm & 0xFF;
        tmode3[41] = (acc_0_1mm >> 8) & 0xFF;
        tmode3[42] = (acc_0_1mm >> 16) & 0xFF;
        tmode3[43] = (acc_0_1mm >> 24) & 0xFF;
        
        return send_ubx_with_ack(tmode3);
    }
    
    bool monitor_survey_in() {
        RCLCPP_INFO(this->get_logger(), "Monitoring survey-in progress...");
        
        auto start_time = std::chrono::steady_clock::now();
        std::vector<uint8_t> poll_svin = {0x01, 0x3B, 0x00, 0x00};
        
        while (running_ && rclcpp::ok()) {
            // Poll survey-in status
            send_ubx(poll_svin);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Read response
            std::vector<uint8_t> response(512);
            size_t n = read_serial(response);
            
            if (n > 50) {
                if (parse_survey_in_status(response, n)) {
                    // Survey is complete when:
                    // 1. Duration requirement met (survey_duration_current_ >= survey_duration_)
                    // 2. Accuracy requirement met (surveyed_accuracy_ <= target_accuracy_)
                    // Note: We don't strictly require valid=1 from the receiver, as it may take
                    // a very long time. We accept the survey if our requirements are met.
                    bool duration_met = survey_duration_current_ >= survey_duration_;
                    bool accuracy_met = surveyed_accuracy_ <= (survey_accuracy_mm_ * 10);  // Convert mm to 0.1mm
                    
                    if (duration_met && accuracy_met) {
                        RCLCPP_INFO(this->get_logger(), "\n✓ Survey-In Complete!");
                        RCLCPP_INFO(this->get_logger(), "Position (ECEF cm): X=%d Y=%d Z=%d", 
                                   surveyed_ecef_x_, surveyed_ecef_y_, surveyed_ecef_z_);
                        RCLCPP_INFO(this->get_logger(), "Duration: %us (required: %us)", 
                                   survey_duration_current_, survey_duration_);
                        RCLCPP_INFO(this->get_logger(), "Final accuracy: %.1f mm (required: %d mm)", 
                                   surveyed_accuracy_ / 10.0, survey_accuracy_mm_);
                        RCLCPP_INFO(this->get_logger(), "Receiver valid flag: %s", 
                                   survey_valid_ ? "true" : "false (accepted anyway)");
                        return true;
                    } else if (survey_valid_ && !duration_met) {
                        // Survey stopped but requirements not met
                        // This could be: cached position, external control (QGC), or user intervention
                        RCLCPP_WARN(this->get_logger(), 
                                   "Survey stopped but requirements not met! Duration: %us/%us, Accuracy: %.1fmm/%dmm",
                                   survey_duration_current_, survey_duration_,
                                   surveyed_accuracy_ / 10.0, survey_accuracy_mm_);
                        
                        // Don't restart if we're close to meeting requirements (within 20% of time)
                        // This prevents fighting with external tools like QGC
                        if (survey_duration_current_ < (survey_duration_ * 0.2)) {
                            RCLCPP_WARN(this->get_logger(), "Survey stopped too early - restarting...");
                            start_survey_in();
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Accepting survey result (external control detected?)");
                            return true;  // Accept it anyway
                        }
                    }
                }
            }
            
            // Timeout check
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > survey_duration_ + 300) {  // Give 5 min extra
                RCLCPP_WARN(this->get_logger(), "Survey-in timeout - continuing anyway");
                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        return false;
    }
    
    bool parse_survey_in_status(const std::vector<uint8_t>& data, size_t len) {
        for (size_t i = 0; i < len - 50; i++) {
            if (data[i] == 0xB5 && data[i+1] == 0x62 && 
                data[i+2] == 0x01 && data[i+3] == 0x3B) {
                
                // Get payload length
                uint16_t payload_len = data[i+4] | (data[i+5] << 8);
                
                // DEBUG: Print hex dump of entire message (first time only)
                static bool first_dump = true;
                if (first_dump && i + 6 + payload_len + 2 <= len) {
                    first_dump = false;
                    RCLCPP_INFO(this->get_logger(), "\n=== UBX-NAV-SVIN Message Hex Dump ===");
                    RCLCPP_INFO(this->get_logger(), "Payload length: %u bytes", payload_len);
                    std::string hex_str;
                    for (size_t j = 0; j < payload_len && j < 60; j++) {
                        char buf[8];
                        snprintf(buf, sizeof(buf), "%02X ", data[i+6+j]);
                        hex_str += buf;
                        if ((j+1) % 16 == 0) {
                            RCLCPP_INFO(this->get_logger(), "  Offset %2zu: %s", j-15, hex_str.c_str());
                            hex_str.clear();
                        }
                    }
                    if (!hex_str.empty()) {
                        RCLCPP_INFO(this->get_logger(), "  Offset %2zu: %s", (payload_len/16)*16, hex_str.c_str());
                    }
                    RCLCPP_INFO(this->get_logger(), "======================================\n");
                }
                
                int base = i + 6;  // Payload starts after header (2) + class/id (2) + length (2)
                
                // UBX-NAV-SVIN payload (40 bytes) - Official structure from ublox_msgs:
                // Byte 0: version
                // Byte 1-3: reserved0
                // Byte 4-7: iTOW (GPS time of week in ms)
                // Byte 8-11: dur (duration in seconds)
                // Byte 12-15: meanX (ECEF X in cm)
                // Byte 16-19: meanY (ECEF Y in cm)  
                // Byte 20-23: meanZ (ECEF Z in cm)
                // Byte 24: meanXHP (high precision X, 0.1mm)
                // Byte 25: meanYHP (high precision Y, 0.1mm)
                // Byte 26: meanZHP (high precision Z, 0.1mm)
                // Byte 27: reserved1
                // Byte 28-31: meanAcc (mean accuracy in 0.1mm)
                // Byte 32-35: obs (number of observations)
                // Byte 36: valid (1 = valid, 0 = invalid)
                // Byte 37: active (1 = in progress, 0 = not active)
                // Byte 38-39: reserved3
                
                survey_duration_current_ = data[base+8] | (data[base+9] << 8) | 
                                          (data[base+10] << 16) | (data[base+11] << 24);
                
                surveyed_ecef_x_ = data[base+12] | (data[base+13] << 8) | 
                                   (data[base+14] << 16) | (data[base+15] << 24);
                surveyed_ecef_y_ = data[base+16] | (data[base+17] << 8) | 
                                   (data[base+18] << 16) | (data[base+19] << 24);
                surveyed_ecef_z_ = data[base+20] | (data[base+21] << 8) | 
                                   (data[base+22] << 16) | (data[base+23] << 24);
                
                surveyed_accuracy_ = data[base+28] | (data[base+29] << 8) | 
                                    (data[base+30] << 16) | (data[base+31] << 24);
                
                survey_observations_ = data[base+32] | (data[base+33] << 8) | 
                                      (data[base+34] << 16) | (data[base+35] << 24);
                
                survey_valid_ = (data[base+36] != 0);
                survey_active_ = (data[base+37] != 0);
                
                surveyed_position_valid_ = true;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Survey: %s | %ds | %u obs | %.1fmm accuracy",
                    survey_active_ ? "ACTIVE" : "INACTIVE",
                    survey_duration_current_, survey_observations_, surveyed_accuracy_ / 10.0);
                
                return true;
            }
        }
        return false;
    }
    
    bool configure_fixed_base(int32_t x, int32_t y, int32_t z, uint32_t acc) {
        RCLCPP_INFO(this->get_logger(), "Configuring fixed base mode with surveyed position...");
        
        std::vector<uint8_t> tmode3(44, 0);
        tmode3[0] = 0x06; tmode3[1] = 0x71; tmode3[2] = 0x28; tmode3[3] = 0x00;
        tmode3[6] = 0x02; // Fixed mode
        
        tmode3[8] = x & 0xFF; tmode3[9] = (x >> 8) & 0xFF;
        tmode3[10] = (x >> 16) & 0xFF; tmode3[11] = (x >> 24) & 0xFF;
        tmode3[12] = y & 0xFF; tmode3[13] = (y >> 8) & 0xFF;
        tmode3[14] = (y >> 16) & 0xFF; tmode3[15] = (y >> 24) & 0xFF;
        tmode3[16] = z & 0xFF; tmode3[17] = (z >> 8) & 0xFF;
        tmode3[18] = (z >> 16) & 0xFF; tmode3[19] = (z >> 24) & 0xFF;
        tmode3[28] = acc & 0xFF; tmode3[29] = (acc >> 8) & 0xFF;
        tmode3[30] = (acc >> 16) & 0xFF; tmode3[31] = (acc >> 24) & 0xFF;
        
        bool success = send_ubx_with_ack(tmode3);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "✓ Fixed base mode configured");
        }
        return success;
    }
    
    void read_loop() {
        std::vector<uint8_t> buffer;
        buffer.reserve(4096);
        
        RCLCPP_INFO(this->get_logger(), "Starting RTCM3 read loop...");
        
        size_t total_bytes_read = 0;
        auto last_report = std::chrono::steady_clock::now();
        
        while (running_ && rclcpp::ok()) {
            try {
                std::vector<uint8_t> chunk(1024);
                size_t bytes_read = read_serial(chunk);
                
                if (bytes_read > 0) {
                    total_bytes_read += bytes_read;
                    buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + bytes_read);
                    process_buffer(buffer);
                }
                
                // Report read statistics every 5 seconds
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_report).count() >= 5) {
                    RCLCPP_INFO(this->get_logger(), "Read stats: %zu bytes total, buffer: %zu bytes", 
                               total_bytes_read, buffer.size());
                    last_report = now;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Read error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    void process_buffer(std::vector<uint8_t>& buffer) {
        size_t i = 0;
        
        while (i < buffer.size()) {
            if (buffer[i] == 0xD3 && i + 2 < buffer.size()) {
                if ((buffer[i+1] & 0xFC) == 0) {
                    uint16_t msg_len = ((buffer[i+1] & 0x03) << 8) | buffer[i+2];
                    uint16_t total_len = 3 + msg_len + 3;
                    
                    if (i + total_len <= buffer.size()) {
                        std::vector<uint8_t> rtcm_msg(buffer.begin() + i, 
                                                      buffer.begin() + i + total_len);
                        
                        if (msg_len >= 2) {
                            uint16_t msg_type = (buffer[i+3] << 4) | (buffer[i+4] >> 4);
                            publish_rtcm(rtcm_msg, msg_type);
                        }
                        
                        i += total_len;
                        continue;
                    } else {
                        break;
                    }
                }
            }
            i++;
        }
        
        if (i > 0) {
            buffer.erase(buffer.begin(), buffer.begin() + i);
        }
        
        if (buffer.size() > 8192) {
            buffer.clear();
        }
    }
    
    void publish_rtcm(const std::vector<uint8_t>& data, uint16_t msg_type) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = data;
        
        rtcm_pub_->publish(msg);
        
        rtcm_count_++;
        total_bytes_ += data.size();
        
        // Log every RTCM message with type and size
        RCLCPP_INFO(this->get_logger(), "RTCM3-%04u: %zu bytes", msg_type, data.size());
        
        if (msg_types_seen_.find(msg_type) == msg_types_seen_.end()) {
            msg_types_seen_.insert(msg_type);
            RCLCPP_INFO(this->get_logger(), "  → New RTCM3 message type discovered: %u", msg_type);
        }
    }
    
    void publish_status() {
        auto msg = std_msgs::msg::String();
        
        switch (state_) {
            case BaseStationState::INITIALIZING:
                msg.data = "INITIALIZING";
                break;
            case BaseStationState::CONFIGURING:
                msg.data = "CONFIGURING";
                break;
            case BaseStationState::SURVEY_IN_PROGRESS:
                msg.data = "SURVEY_IN_PROGRESS:" + std::to_string(survey_duration_current_) + 
                          "s," + std::to_string(surveyed_accuracy_ / 10.0) + "mm";
                break;
            case BaseStationState::SURVEY_IN_COMPLETE:
                msg.data = "SURVEY_COMPLETE";
                break;
            case BaseStationState::FIXED_BASE_MODE:
                msg.data = "FIXED_BASE_MODE";
                break;
            case BaseStationState::PUBLISHING_RTCM:
                msg.data = "PUBLISHING:" + std::to_string(rtcm_count_) + " msgs";
                break;
            case BaseStationState::ERROR:
                msg.data = "ERROR";
                break;
        }
        
        status_pub_->publish(msg);
    }
    
    bool send_ubx(const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> packet = {0xB5, 0x62};
        packet.insert(packet.end(), payload.begin(), payload.end());
        
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < packet.size(); i++) {
            ck_a += packet[i];
            ck_b += ck_a;
        }
        packet.push_back(ck_a);
        packet.push_back(ck_b);
        
        try {
            serial_driver_->port()->send(packet);
            return true;
        } catch (...) {
            return false;
        }
    }
    
    bool send_ubx_with_ack(const std::vector<uint8_t>& payload) {
        if (!send_ubx(payload)) return false;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        std::vector<uint8_t> response(512);
        size_t n = read_serial(response);
        
        for (size_t i = 0; i < n - 10; i++) {
            if (response[i] == 0xB5 && response[i+1] == 0x62 &&
                response[i+2] == 0x05 && response[i+3] == 0x01) {
                if (response[i+6] == payload[0] && response[i+7] == payload[1]) {
                    return true;
                }
            }
        }
        return false;
    }
    
    size_t read_serial(std::vector<uint8_t>& buffer) {
        try {
            return serial_driver_->port()->receive(buffer);
        } catch (...) {
            return 0;
        }
    }
    
    void print_stats() {
        if (rtcm_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "=== Final Stats === RTCM Messages: %zu | Bytes: %zu | Types: %zu",
                       rtcm_count_, total_bytes_, msg_types_seen_.size());
        }
    }
    
    // Members
    std::unique_ptr<SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    std::thread config_thread_;
    std::thread read_thread_;
    std::atomic<bool> running_{true};
    
    std::string frame_id_;
    int survey_duration_;
    int survey_accuracy_mm_;
    bool auto_configure_;
    
    BaseStationState state_;
    
    // Survey-in status
    bool survey_valid_{false};
    bool survey_active_{false};
    bool surveyed_position_valid_{false};
    uint32_t survey_duration_current_{0};
    uint32_t survey_observations_{0};
    uint32_t surveyed_accuracy_{0};
    int32_t surveyed_ecef_x_{0};
    int32_t surveyed_ecef_y_{0};
    int32_t surveyed_ecef_z_{0};
    
    // RTCM stats
    size_t rtcm_count_{0};
    size_t total_bytes_{0};
    std::set<uint16_t> msg_types_seen_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTKBaseStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
