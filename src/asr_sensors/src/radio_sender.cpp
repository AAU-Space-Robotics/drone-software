#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <serial_driver/serial_driver.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <sstream>
#include <regex>

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

// Simple frame structure: [START_BYTE][LENGTH_HIGH][LENGTH_LOW][PAYLOAD][CHECKSUM]
constexpr uint8_t FRAME_START = 0xAA;

class RadioSender : public rclcpp::Node {
public:
    RadioSender() : Node("radio_sender"), io_context_(new drivers::common::IoContext(2)) {
        // Configure serial port
        SerialPortConfig config(57600, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        
        // Create serial driver
        serial_driver_ = std::make_unique<SerialDriver>(*io_context_);
        
        try {
            serial_driver_->init_port("/dev/ttyUSB2", config);
            if (serial_driver_->port()) {
                serial_driver_->port()->open();
                RCLCPP_INFO(this->get_logger(), "Serial Port initialized on /dev/ttyUSB0 at 57600 baud");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create serial port");
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
            return;
        }
        
        // Query radio information at startup
        query_radio_info();
        
        // Create publisher for radio diagnostics
        radio_diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/thyra/radio/tx/diagnostics", 10);
        
        // Create timer to send data periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RadioSender::send_message, this));
        
        // Create timer for statistics (every 10 seconds)
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&RadioSender::print_stats, this));
        
        // Create timer for radio diagnostics (every 5 seconds)
        radio_diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RadioSender::publish_radio_diagnostics, this));
    }
    
    ~RadioSender() {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        print_stats();
    }

private:
    void query_radio_info() {
        RCLCPP_INFO(this->get_logger(), "Querying radio information...");
        
        // Enter AT command mode: send +++, wait 1 second
        std::string cmd_mode = "+++";
        std::vector<uint8_t> cmd_buffer(cmd_mode.begin(), cmd_mode.end());
        
        try {
            serial_driver_->port()->send(cmd_buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            // Query radio version (ATI)
            send_at_command("ATI\r\n", "Radio Version");
            
            // Query board ID (ATI2)
            send_at_command("ATI2\r\n", "Board ID");
            
            // Query radio statistics (ATI7)
            send_at_command("ATI7\r\n", "RSSI Stats");
            
            // Exit AT command mode (ATO)
            std::string exit_cmd = "ATO\r\n";
            std::vector<uint8_t> exit_buffer(exit_cmd.begin(), exit_cmd.end());
            serial_driver_->port()->send(exit_buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            RCLCPP_INFO(this->get_logger(), "Radio information query complete");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to query radio info: %s", e.what());
        }
    }
    
    void send_at_command(const std::string& cmd, const std::string& desc) {
        std::vector<uint8_t> buffer(cmd.begin(), cmd.end());
        serial_driver_->port()->send(buffer);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // Try to read response
        std::vector<uint8_t> response(512);
        size_t bytes = serial_driver_->port()->receive(response);
        if (bytes > 0) {
            std::string resp(response.begin(), response.begin() + bytes);
            // Remove control characters
            resp.erase(std::remove_if(resp.begin(), resp.end(), 
                [](char c) { return c == '\r' || c == '\n' || c == '\0'; }), resp.end());
            if (!resp.empty()) {
                RCLCPP_INFO(this->get_logger(), "%s: %s", desc.c_str(), resp.c_str());
            }
        }
    }
    
    std::vector<uint8_t> create_frame(const std::string& payload) {
        std::vector<uint8_t> frame;
        uint16_t length = payload.size();
        
        // Add frame header
        frame.push_back(FRAME_START);
        frame.push_back((length >> 8) & 0xFF);  // Length high byte
        frame.push_back(length & 0xFF);         // Length low byte
        
        // Add payload
        frame.insert(frame.end(), payload.begin(), payload.end());
        
        // Calculate simple checksum (XOR of all bytes)
        uint8_t checksum = 0;
        for (size_t i = 1; i < frame.size(); i++) {
            checksum ^= frame[i];
        }
        frame.push_back(checksum);
        
        return frame;
    }
    
    void send_message() {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
            std::string msg = "Hello from sender! Count: " + std::to_string(count_++);
            
            // Create framed message
            std::vector<uint8_t> frame = create_frame(msg);
            
            try {
                size_t bytes_sent = serial_driver_->port()->send(frame);
                total_bytes_sent_ += bytes_sent;
                messages_sent_++;
                
                if (count_ % 10 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Sent [%d]: %s (%zu bytes, %zu frame)", 
                                count_ - 1, msg.c_str(), msg.size(), bytes_sent);
                }
            } catch (const std::exception& e) {
                send_errors_++;
                RCLCPP_ERROR(this->get_logger(), "Failed to send: %s", e.what());
            }
        }
    }
    
    void print_stats() {
        if (messages_sent_ > 0) {
            double avg_msg_size = static_cast<double>(total_bytes_sent_) / messages_sent_;
            double data_rate_bps = (total_bytes_sent_ * 8.0) / 
                                   (count_ > 0 ? count_ : 1);  // bits per second
            
            RCLCPP_INFO(this->get_logger(), 
                       "=== Sender Stats === Messages: %zu | Bytes: %zu | Avg Size: %.1f | "
                       "Data Rate: %.0f bps | Errors: %zu",
                       messages_sent_, total_bytes_sent_, avg_msg_size, 
                       data_rate_bps, send_errors_);
        }
    }
    
    void publish_radio_diagnostics() {
        // Query RSSI stats
        std::string rssi_data = query_rssi_stats();
        if (rssi_data.empty()) {
            return;
        }
        
        auto msg = diagnostic_msgs::msg::DiagnosticStatus();
        msg.name = "Radio TX";
        msg.hardware_id = "SiK_Radio_TX";
        msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        
        // Parse RSSI data: "L/R RSSI: 205/207  L/R noise: 53/60 pkts: 353  txe=0 rxe=1..."
        std::regex rssi_regex(R"(L/R RSSI: (\d+)/(\d+)\s+L/R noise: (\d+)/(\d+)\s+pkts: (\d+)\s+txe=(\d+)\s+rxe=(\d+))");
        std::smatch matches;
        
        if (std::regex_search(rssi_data, matches, rssi_regex)) {
            diagnostic_msgs::msg::KeyValue kv;
            
            kv.key = "local_rssi";
            kv.value = matches[1].str();
            msg.values.push_back(kv);
            
            kv.key = "remote_rssi";
            kv.value = matches[2].str();
            msg.values.push_back(kv);
            
            kv.key = "local_noise";
            kv.value = matches[3].str();
            msg.values.push_back(kv);
            
            kv.key = "remote_noise";
            kv.value = matches[4].str();
            msg.values.push_back(kv);
            
            kv.key = "packets";
            kv.value = matches[5].str();
            msg.values.push_back(kv);
            
            kv.key = "tx_errors";
            kv.value = matches[6].str();
            msg.values.push_back(kv);
            
            kv.key = "rx_errors";
            kv.value = matches[7].str();
            msg.values.push_back(kv);
            
            // Add software stats
            kv.key = "messages_sent";
            kv.value = std::to_string(messages_sent_);
            msg.values.push_back(kv);
            
            kv.key = "bytes_sent";
            kv.value = std::to_string(total_bytes_sent_);
            msg.values.push_back(kv);
            
            kv.key = "send_errors";
            kv.value = std::to_string(send_errors_);
            msg.values.push_back(kv);
            
            // Set warning level if errors detected
            int tx_err = std::stoi(matches[6].str());
            int rx_err = std::stoi(matches[7].str());
            if (tx_err > 0 || rx_err > 0 || send_errors_ > 0) {
                msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg.message = "Errors detected";
            } else {
                msg.message = "Radio link OK";
            }
            
            radio_diag_pub_->publish(msg);
        }
    }
    
    std::string query_rssi_stats() {
        try {
            // Enter AT command mode
            std::string cmd_mode = "+++";
            std::vector<uint8_t> cmd_buffer(cmd_mode.begin(), cmd_mode.end());
            serial_driver_->port()->send(cmd_buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(1200));
            
            // Query ATI7
            std::string cmd = "ATI7\r\n";
            std::vector<uint8_t> buffer(cmd.begin(), cmd.end());
            serial_driver_->port()->send(buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // Read response
            std::vector<uint8_t> response(512);
            size_t bytes = serial_driver_->port()->receive(response);
            
            // Exit AT command mode
            std::string exit_cmd = "ATO\r\n";
            std::vector<uint8_t> exit_buffer(exit_cmd.begin(), exit_cmd.end());
            serial_driver_->port()->send(exit_buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            if (bytes > 0) {
                return std::string(response.begin(), response.begin() + bytes);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to query RSSI: %s", e.what());
        }
        return "";
    }
    
    std::unique_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr radio_diag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr radio_diag_timer_;
    int count_ = 0;
    size_t total_bytes_sent_ = 0;
    size_t messages_sent_ = 0;
    size_t send_errors_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadioSender>());
    rclcpp::shutdown();
    return 0;
}