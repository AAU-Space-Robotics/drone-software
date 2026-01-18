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
#include <sstream>
#include <regex>

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

// Frame structure constants
constexpr uint8_t FRAME_START = 0xAA;
constexpr size_t FRAME_HEADER_SIZE = 4;  // START + LENGTH_HIGH + LENGTH_LOW + CHECKSUM

class RadioReceiver : public rclcpp::Node {
public:
    RadioReceiver() : Node("radio_receiver"), io_context_(new drivers::common::IoContext(2)) {
        // Configure serial port
        SerialPortConfig config(57600, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        
        // Create serial driver
        serial_driver_ = std::make_unique<SerialDriver>(*io_context_);
        
        try {
            serial_driver_->init_port("/dev/ttyUSB1", config);
            if (serial_driver_->port()) {
                serial_driver_->port()->open();
                RCLCPP_INFO(this->get_logger(), "Serial Port initialized on /dev/ttyUSB1 at 57600 baud");
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
            "/thyra/radio/rx/diagnostics", 10);
        
        // Create timer to read data more frequently (10ms instead of 100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RadioReceiver::read_message, this));
        
        // Create timer for statistics (every 10 seconds)
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&RadioReceiver::print_stats, this));
        
        // Create timer for radio diagnostics (every 5 seconds)
        radio_diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RadioReceiver::publish_radio_diagnostics, this));
    }
    
    ~RadioReceiver() {
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
    
    bool parse_frame(std::vector<uint8_t>& raw_buffer, std::string& payload) {
        // Look for frame start byte
        while (!raw_buffer.empty() && raw_buffer[0] != FRAME_START) {
            raw_buffer.erase(raw_buffer.begin());
            frame_sync_errors_++;
        }
        
        // Need at least header bytes
        if (raw_buffer.size() < FRAME_HEADER_SIZE) {
            return false;
        }
        
        // Extract length
        uint16_t length = (static_cast<uint16_t>(raw_buffer[1]) << 8) | raw_buffer[2];
        
        // Check if we have complete frame
        size_t total_frame_size = FRAME_HEADER_SIZE + length;
        if (raw_buffer.size() < total_frame_size) {
            return false;
        }
        
        // Verify checksum
        uint8_t received_checksum = raw_buffer[total_frame_size - 1];
        uint8_t calculated_checksum = 0;
        for (size_t i = 1; i < total_frame_size - 1; i++) {
            calculated_checksum ^= raw_buffer[i];
        }
        
        if (received_checksum != calculated_checksum) {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch! Expected 0x%02X, got 0x%02X", 
                       calculated_checksum, received_checksum);
            raw_buffer.erase(raw_buffer.begin());  // Remove bad start byte
            checksum_errors_++;
            return false;
        }
        
        // Extract payload
        payload = std::string(raw_buffer.begin() + 3, raw_buffer.begin() + 3 + length);
        
        // Remove processed frame from buffer
        raw_buffer.erase(raw_buffer.begin(), raw_buffer.begin() + total_frame_size);
        
        return true;
    }
    
    void read_message() {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
            std::vector<uint8_t> buffer(512);
            try {
                size_t bytes_read = serial_driver_->port()->receive(buffer);
                if (bytes_read > 0) {
                    total_bytes_received_ += bytes_read;
                    
                    // Append to raw buffer
                    raw_buffer_.insert(raw_buffer_.end(), buffer.begin(), buffer.begin() + bytes_read);
                    
                    // Prevent buffer overflow
                    if (raw_buffer_.size() > 4096) {
                        RCLCPP_WARN(this->get_logger(), "Raw buffer overflow! Clearing buffer.");
                        buffer_overflows_++;
                        raw_buffer_.clear();
                    }
                    
                    // Try to parse frames
                    std::string payload;
                    while (parse_frame(raw_buffer_, payload)) {
                        messages_received_++;
                        
                        // Track packet loss
                        size_t count_pos = payload.find("Count: ");
                        if (count_pos != std::string::npos) {
                            try {
                                int msg_count = std::stoi(payload.substr(count_pos + 7));
                                if (last_received_count_ >= 0) {
                                    int expected = last_received_count_ + 1;
                                    if (msg_count > expected) {
                                        int lost = msg_count - expected;
                                        messages_lost_ += lost;
                                        if (lost > 5) {
                                            RCLCPP_WARN(this->get_logger(), 
                                                       "Large packet loss! Expected %d, got %d (%d lost)", 
                                                       expected, msg_count, lost);
                                        }
                                    }
                                }
                                last_received_count_ = msg_count;
                            } catch (...) {}
                        }
                        
                        // Log every 10th message
                        if (messages_received_ % 10 == 0) {
                            RCLCPP_INFO(this->get_logger(), "Received [%zu]: %s", 
                                       messages_received_, payload.c_str());
                        }
                    }
                }
            } catch (const std::exception& e) {
                receive_errors_++;
                RCLCPP_ERROR(this->get_logger(), "Failed to receive: %s", e.what());
            }
        }
    }
    
    void print_stats() {
        if (messages_received_ > 0 || messages_lost_ > 0) {
            double packet_loss_rate = (messages_lost_ > 0) ? 
                (100.0 * messages_lost_) / (messages_received_ + messages_lost_) : 0.0;
            
            if (packet_loss_rate > 20.0) {
                RCLCPP_WARN(this->get_logger(), 
                           "=== Stats === RX: %zu | Lost: %zu (%.1f%%) | Bytes: %zu | "
                           "Checksum Err: %zu | Sync Err: %zu | Overflows: %zu",
                           messages_received_, messages_lost_, packet_loss_rate,
                           total_bytes_received_, checksum_errors_, frame_sync_errors_, buffer_overflows_);
            } else {
                RCLCPP_INFO(this->get_logger(), 
                           "=== Stats === RX: %zu | Lost: %zu (%.1f%%) | Bytes: %zu | "
                           "Checksum Err: %zu | Sync Err: %zu | Overflows: %zu",
                           messages_received_, messages_lost_, packet_loss_rate,
                           total_bytes_received_, checksum_errors_, frame_sync_errors_, buffer_overflows_);
            }
        }
    }
    
    void publish_radio_diagnostics() {
        // Query RSSI stats
        std::string rssi_data = query_rssi_stats();
        if (rssi_data.empty()) {
            return;
        }
        
        auto msg = diagnostic_msgs::msg::DiagnosticStatus();
        msg.name = "Radio RX";
        msg.hardware_id = "SiK_Radio_RX";
        msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        
        // Parse RSSI data
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
            kv.key = "messages_received";
            kv.value = std::to_string(messages_received_);
            msg.values.push_back(kv);
            
            kv.key = "messages_lost";
            kv.value = std::to_string(messages_lost_);
            msg.values.push_back(kv);
            
            kv.key = "bytes_received";
            kv.value = std::to_string(total_bytes_received_);
            msg.values.push_back(kv);
            
            kv.key = "checksum_errors";
            kv.value = std::to_string(checksum_errors_);
            msg.values.push_back(kv);
            
            kv.key = "frame_sync_errors";
            kv.value = std::to_string(frame_sync_errors_);
            msg.values.push_back(kv);
            
            // Calculate packet loss rate
            double loss_rate = (messages_lost_ > 0) ? 
                (100.0 * messages_lost_) / (messages_received_ + messages_lost_) : 0.0;
            kv.key = "packet_loss_percent";
            kv.value = std::to_string(loss_rate);
            msg.values.push_back(kv);
            
            // Set warning/error level based on conditions
            int tx_err = std::stoi(matches[6].str());
            int rx_err = std::stoi(matches[7].str());
            if (loss_rate > 50.0 || checksum_errors_ > 10) {
                msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                msg.message = "High packet loss or errors";
            } else if (tx_err > 0 || rx_err > 0 || loss_rate > 20.0) {
                msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                msg.message = "Degraded link quality";
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
    std::vector<uint8_t> raw_buffer_;
    size_t total_bytes_received_ = 0;
    size_t messages_received_ = 0;
    size_t messages_lost_ = 0;
    size_t receive_errors_ = 0;
    size_t buffer_overflows_ = 0;
    size_t checksum_errors_ = 0;
    size_t frame_sync_errors_ = 0;
    int last_received_count_ = -1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadioReceiver>());
    rclcpp::shutdown();
    return 0;
}