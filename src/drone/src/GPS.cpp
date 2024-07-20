#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <drone/msg/gps_data.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <memory>

using namespace std::chrono_literals;

class SerialReader : public rclcpp::Node
{
public:
    SerialReader()
    : Node("serial_reader")
    {
        publisher_ = this->create_publisher<drone::msg::GPSData>("gps_data", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&SerialReader::timer_callback, this));
        
        serial_port_ = "/dev/ttyUSB0";  // Adjust to your serial port
        baud_rate_ = 115200;  // Adjust to your baud rate

        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port %s", serial_port_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "SerialReader node started, reading from %s at %d baud rate.",
                    serial_port_.c_str(), baud_rate_);
    }

private:
    void timer_callback()
    {
        if (serial_.available())
        {
            try {
                std::string line = serial_.readline();
                RCLCPP_INFO(this->get_logger(), "Read from serial: %s", line.c_str());

                // Parse the GPS data from the string
                if (line.find("<gps>") != std::string::npos && line.find("</gps>") != std::string::npos)
                {
                    auto data = parse_gps_data(line);
                    if (data)
                    {
                        auto msg = drone::msg::GPSData();
                        msg.lat = data->at("lat");
                        msg.lon = data->at("lon");
                        msg.date = data->at("date");
                        msg.time = data->at("time");
                        msg.course = data->at("course");
                        msg.speed = data->at("speed");
                        msg.pdop = data->at("pdop");
                        msg.hdop = data->at("hdop");
                        publisher_->publish(msg);
                    }
                }
            } catch (serial::IOException &e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }

    std::shared_ptr<std::unordered_map<std::string, std::string>> parse_gps_data(const std::string& data)
    {
        auto gps_data = std::make_shared<std::unordered_map<std::string, std::string>>();

        auto parse_tag_value = [&data](const std::string& tag) -> std::string {
            std::string start_tag = "<" + tag + ">";
            std::string end_tag = "</" + tag + ">";
            size_t start_pos = data.find(start_tag);
            size_t end_pos = data.find(end_tag);
            if (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                start_pos += start_tag.length();
                return data.substr(start_pos, end_pos - start_pos);
            }
            return "";
        };

        auto parse_float_tag_value = [&](const std::string& tag) -> float {
            try
            {
                return std::stof(parse_tag_value(tag));
            }
            catch (...)
            {
                return NAN;
            }
        };

        (*gps_data)["lat"] = parse_tag_value("lat");
        (*gps_data)["lon"] = parse_tag_value("lon");
        (*gps_data)["date"] = parse_tag_value("date");
        (*gps_data)["time"] = parse_tag_value("time");
        (*gps_data)["course"] = parse_tag_value("course");
        (*gps_data)["speed"] = parse_tag_value("speed");
        (*gps_data)["pdop"] = parse_tag_value("pdop");
        (*gps_data)["hdop"] = parse_tag_value("hdop");

        return gps_data;
    }

    rclcpp::Publisher<drone::msg::GPSData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
    std::string serial_port_;
    uint32_t baud_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialReader>());
    rclcpp::shutdown();
    return 0;
}
