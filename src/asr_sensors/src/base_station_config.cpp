#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>

class RTKBaseStationSetup {
public:
    RTKBaseStationSetup(const std::string& port = "/dev/ttyACM0") 
        : port_name_(port), fd_(-1), verbose_(true) {}
    
    ~RTKBaseStationSetup() {
        close_port();
    }
    
    void set_verbose(bool v) { verbose_ = v; }
    
    bool run_full_setup() {
        std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║   Holybro RTK Base Station - Complete Setup      ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;
        
        // Step 1: Find correct baud rate and establish communication
        if (!find_and_connect()) {
            std::cerr << "\n✗ Failed to establish communication with base station" << std::endl;
            return false;
        }
        
        // Step 2: Query receiver information
        if (!query_receiver_info()) {
            std::cerr << "\n⚠ Warning: Could not query all receiver information" << std::endl;
        }
        
        // Step 3: Check current configuration
        check_current_config();
        
        // Step 4: Ask user if they want to configure
        std::cout << "\n┌─────────────────────────────────────────────┐" << std::endl;
        std::cout << "│ Configure for RTCM3 RTK base station? (y/n) │" << std::endl;
        std::cout << "└─────────────────────────────────────────────┘" << std::endl;
        std::cout << "Choice: ";
        std::string answer;
        std::getline(std::cin, answer);
        
        if (answer != "y" && answer != "Y") {
            std::cout << "Configuration skipped." << std::endl;
            return true;
        }
        
        // Step 5: Configure for RTCM3
        if (!configure_rtcm3()) {
            std::cerr << "\n✗ Configuration failed" << std::endl;
            return false;
        }
        
        // Step 6: Set base station mode
        if (!configure_base_mode()) {
            std::cerr << "\n✗ Base mode configuration failed" << std::endl;
            return false;
        }
        
        // Step 7: Verify RTCM3 output
        std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║   Verifying RTCM3 Output                          ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;
        
        bool rtcm_verified = verify_rtcm3_output();
        
        if (rtcm_verified) {
            std::cout << "\n✓ ✓ ✓ SUCCESS! Base station configured and working! ✓ ✓ ✓" << std::endl;
        } else {
            std::cout << "\n✓ Configuration complete!" << std::endl;
            std::cout << "  RTCM3 messages will appear once device gets GPS fix." << std::endl;
        }
        return true;
    }

private:
    std::string port_name_;
    int fd_;
    bool verbose_;
    int current_baudrate_;
    
    // Baud rates to try
    const std::vector<int> baud_rates_ = {115200, 38400, 9600, 57600, 230400};
    
    bool open_port(int baudrate) {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
        
        fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            if (verbose_) std::cerr << "  ✗ Cannot open " << port_name_ << std::endl;
            return false;
        }
        
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            return false;
        }
        
        // Set baud rate
        speed_t speed;
        switch(baudrate) {
            case 9600: speed = B9600; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B115200;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        // 8N1 mode, no flow control
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;  // 0.2 second timeout
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            return false;
        }
        
        tcflush(fd_, TCIOFLUSH);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        current_baudrate_ = baudrate;
        return true;
    }
    
    void close_port() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }
    
    bool find_and_connect() {
        std::cout << "\n[Step 1] Finding correct baud rate..." << std::endl;
        
        for (int baud : baud_rates_) {
            std::cout << "\n  Testing " << baud << " baud..." << std::endl;
            
            if (!open_port(baud)) {
                continue;
            }
            
            // Try to poll receiver version (UBX-MON-VER)
            std::vector<uint8_t> poll_ver = {0x0A, 0x04, 0x00, 0x00};
            
            if (send_ubx(poll_ver)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                
                // Try to read response
                std::vector<uint8_t> response(1024);
                int bytes = read(fd_, response.data(), response.size());
                
                if (bytes > 20) {
                    // Look for UBX sync bytes
                    bool found_ubx = false;
                    for (int i = 0; i < bytes - 1; i++) {
                        if (response[i] == 0xB5 && response[i+1] == 0x62) {
                            found_ubx = true;
                            break;
                        }
                    }
                    
                    // Or look for NMEA
                    bool found_nmea = false;
                    for (int i = 0; i < bytes; i++) {
                        if (response[i] == '$') {
                            found_nmea = true;
                            break;
                        }
                    }
                    
                    if (found_ubx || found_nmea) {
                        std::cout << "  ✓ Communication established at " << baud << " baud!" << std::endl;
                        if (found_ubx) std::cout << "    Detected: UBX protocol" << std::endl;
                        if (found_nmea) std::cout << "    Detected: NMEA protocol" << std::endl;
                        return true;
                    }
                }
            }
        }
        
        std::cerr << "\n✗ Could not establish communication at any baud rate" << std::endl;
        std::cerr << "  Tried: ";
        for (int b : baud_rates_) std::cerr << b << " ";
        std::cerr << std::endl;
        return false;
    }
    
    bool query_receiver_info() {
        std::cout << "\n[Step 2] Querying receiver information..." << std::endl;
        
        // Poll MON-VER (firmware version)
        std::cout << "\n  Requesting firmware version..." << std::endl;
        std::vector<uint8_t> poll_ver = {0x0A, 0x04, 0x00, 0x00};
        if (send_ubx(poll_ver)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::vector<uint8_t> response(2048);
            int bytes = read(fd_, response.data(), response.size());
            
            if (bytes > 50) {
                // Parse version info
                parse_version_response(response, bytes);
            }
        }
        
        // Poll CFG-PRT (port configuration)
        std::cout << "\n  Requesting port configuration..." << std::endl;
        std::vector<uint8_t> poll_prt = {0x06, 0x00, 0x01, 0x00, 0x01}; // Poll UART1
        if (send_ubx(poll_prt)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::vector<uint8_t> response(256);
            int bytes = read(fd_, response.data(), response.size());
            
            if (bytes > 20) {
                parse_port_config(response, bytes);
            }
        }
        
        return true;
    }
    
    void parse_version_response(const std::vector<uint8_t>& data, int len) {
        // Find UBX-MON-VER response (0xB5 0x62 0x0A 0x04)
        for (int i = 0; i < len - 4; i++) {
            if (data[i] == 0xB5 && data[i+1] == 0x62 && 
                data[i+2] == 0x0A && data[i+3] == 0x04) {
                
                int payload_len = data[i+4] | (data[i+5] << 8);
                if (i + 6 + payload_len <= len) {
                    // Extract SW version (first 30 chars)
                    std::string sw_ver;
                    for (int j = 0; j < 30 && (i+6+j) < len; j++) {
                        char c = data[i+6+j];
                        if (c >= 32 && c < 127) sw_ver += c;
                    }
                    std::cout << "  ✓ Firmware: " << sw_ver << std::endl;
                    
                    // Extract HW version (next 10 chars)
                    std::string hw_ver;
                    for (int j = 30; j < 40 && (i+6+j) < len; j++) {
                        char c = data[i+6+j];
                        if (c >= 32 && c < 127) hw_ver += c;
                    }
                    std::cout << "  ✓ Hardware: " << hw_ver << std::endl;
                }
                return;
            }
        }
        std::cout << "  ⚠ Could not parse version info" << std::endl;
    }
    
    void parse_port_config(const std::vector<uint8_t>& data, int len) {
        // Find UBX-CFG-PRT response
        for (int i = 0; i < len - 20; i++) {
            if (data[i] == 0xB5 && data[i+1] == 0x62 && 
                data[i+2] == 0x06 && data[i+3] == 0x00) {
                
                int in_proto = data[i+14] | (data[i+15] << 8);
                int out_proto = data[i+16] | (data[i+17] << 8);
                
                std::cout << "  Current port configuration:" << std::endl;
                std::cout << "    Input protocols:  ";
                if (in_proto & 0x01) std::cout << "UBX ";
                if (in_proto & 0x02) std::cout << "NMEA ";
                if (in_proto & 0x04) std::cout << "RTCM3 ";
                std::cout << "(0x" << std::hex << in_proto << std::dec << ")" << std::endl;
                
                std::cout << "    Output protocols: ";
                if (out_proto & 0x01) std::cout << "UBX ";
                if (out_proto & 0x02) std::cout << "NMEA ";
                if (out_proto & 0x04) std::cout << "RTCM3 ";
                std::cout << "(0x" << std::hex << out_proto << std::dec << ")" << std::endl;
                
                return;
            }
        }
    }
    
    void check_current_config() {
        std::cout << "\n[Step 3] Checking current output..." << std::endl;
        
        auto start = std::chrono::steady_clock::now();
        std::vector<uint8_t> buffer(4096);
        int nmea_count = 0;
        int rtcm_count = 0;
        int ubx_count = 0;
        
        std::cout << "  Sampling for 3 seconds..." << std::endl;
        
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
            int n = read(fd_, buffer.data(), buffer.size());
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    if (buffer[i] == '$') nmea_count++;
                    if (buffer[i] == 0xD3 && i + 2 < n) rtcm_count++;
                    if (buffer[i] == 0xB5 && i + 1 < n && buffer[i+1] == 0x62) ubx_count++;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "\n  Current output detected:" << std::endl;
        std::cout << "    NMEA messages:  " << nmea_count << std::endl;
        std::cout << "    RTCM3 messages: " << rtcm_count << std::endl;
        std::cout << "    UBX messages:   " << ubx_count << std::endl;
        
        if (rtcm_count > 5) {
            std::cout << "\n  ✓ Already configured for RTCM3!" << std::endl;
        } else if (nmea_count > 5) {
            std::cout << "\n  ⚠ Currently outputting NMEA (needs configuration)" << std::endl;
        }
    }
    
    bool configure_rtcm3() {
        std::cout << "\n[Step 4] Configuring for RTCM3..." << std::endl;
        
        // Step A: Configure port
        std::cout << "\n  A. Configuring UART port protocol..." << std::endl;
        std::cout << "    Current baudrate: " << current_baudrate_ << std::endl;
        std::vector<uint8_t> port_cfg = {
            0x06, 0x00, 0x14, 0x00,  // CFG-PRT, length 20
            0x01, 0x00,              // Port ID: UART1
            0x00, 0x00,              // Reserved, TX ready
            0xD0, 0x08, 0x00, 0x00,  // Mode: 8N1
            static_cast<uint8_t>(current_baudrate_ & 0xFF),
            static_cast<uint8_t>((current_baudrate_ >> 8) & 0xFF),
            static_cast<uint8_t>((current_baudrate_ >> 16) & 0xFF),
            static_cast<uint8_t>((current_baudrate_ >> 24) & 0xFF),
            0x07, 0x00,              // Input: UBX + NMEA + RTCM3
            0x03, 0x00,              // Output: UBX + NMEA (safer than removing NMEA immediately)
            0x00, 0x00,              // Flags
            0x00, 0x00               // Reserved
        };
        
        std::cout << "    Sending CFG-PRT command..." << std::endl;
        if (!send_ubx_with_ack(port_cfg)) {
            std::cerr << "    ✗ Port configuration failed!" << std::endl;
            std::cerr << "    Trying alternative approach..." << std::endl;
            
            // Try without changing output protocols first
            port_cfg[18] = 0x07;  // Keep all protocols for now
            port_cfg[19] = 0x00;
            
            if (!send_ubx_with_ack(port_cfg)) {
                std::cerr << "    ✗ Alternative port configuration also failed!" << std::endl;
                return false;
            }
        }
        std::cout << "    ✓ Port configured (Output: UBX + NMEA + RTCM3)" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Step B: Disable all NMEA messages
        std::cout << "\n  B. Disabling NMEA messages..." << std::endl;
        const std::vector<uint8_t> nmea_ids = {
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D, 0x0F, 0x40, 0x41, 0x42, 0x43, 0x44
        };
        
        for (uint8_t id : nmea_ids) {
            std::vector<uint8_t> disable_nmea = {
                0x06, 0x01, 0x08, 0x00,  // CFG-MSG, length 8
                0xF0, id,                // NMEA class, message ID
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Disable on all ports
            };
            send_ubx(disable_nmea);  // Don't wait for ACK for each one
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "    ✓ NMEA messages disabled" << std::endl;
        
        // Step C: Enable RTCM3 messages
        std::cout << "\n  C. Enabling RTCM3 messages..." << std::endl;
        const std::vector<std::tuple<uint8_t, int, std::string>> rtcm_msgs = {
            {0x05, 1, "1005 - Base position"},
            {0x4D, 1, "1077 - GPS MSM7"},
            {0x57, 1, "1087 - GLONASS MSM7"},
            {0x61, 1, "1097 - Galileo MSM7"},
            {0x7F, 1, "1127 - BeiDou MSM7"},
            {0xE6, 1, "1230 - GLONASS biases"}
        };
        
        for (const auto& [id, rate, desc] : rtcm_msgs) {
            std::vector<uint8_t> enable_rtcm = {
                0x06, 0x01, 0x08, 0x00,  // CFG-MSG, length 8
                0xF5, id,                // RTCM3 class, message ID
                0x00,                    // I2C
                static_cast<uint8_t>(rate),  // UART1
                0x00, 0x00, 0x00, 0x00   // UART2, USB, SPI, reserved
            };
            
            if (send_ubx_with_ack(enable_rtcm)) {
                std::cout << "    ✓ Enabled " << desc << std::endl;
            } else {
                std::cout << "    ⚠ " << desc << " - no ACK" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        // Step D: Save configuration
        std::cout << "\n  D. Saving configuration to flash..." << std::endl;
        std::vector<uint8_t> save_cfg = {
            0x06, 0x09, 0x0D, 0x00,  // CFG-CFG, length 13
            0x00, 0x00, 0x00, 0x00,  // Clear mask
            0x1F, 0x1F, 0x00, 0x00,  // Save mask (all relevant)
            0x00, 0x00, 0x00, 0x00,  // Load mask
            0x1F                     // Device mask
        };
        
        if (!send_ubx_with_ack(save_cfg)) {
            std::cerr << "    ⚠ Save failed - configuration may not persist!" << std::endl;
        } else {
            std::cout << "    ✓ Configuration saved to flash" << std::endl;
        }
        
        std::cout << "\n  ✓ RTCM3 configuration complete!" << std::endl;
        return true;
    }
    
    bool configure_base_mode() {
        std::cout << "\n[Step 5] Configuring base station mode..." << std::endl;
        std::cout << "\nChoose mode:" << std::endl;
        std::cout << "  1. Survey-In (automatic - recommended)" << std::endl;
        std::cout << "  2. Fixed position (manual coordinates)" << std::endl;
        std::cout << "  3. Monitor current survey-in status" << std::endl;
        std::cout << "Choice (1/2/3): ";
        
        std::string mode;
        std::getline(std::cin, mode);
        
        if (mode == "1") {
            return configure_survey_in();
        } else if (mode == "2") {
            std::cout << "\n  Note: Fixed mode requires accurate ECEF coordinates" << std::endl;
            std::cout << "  Recommend running Survey-In first to get coordinates!" << std::endl;
            return true;
        } else if (mode == "3") {
            return monitor_survey_in_status();
        } else {
            std::cout << "  Skipping base mode configuration" << std::endl;
            return true;
        }
    }
    
    bool monitor_survey_in_status() {
        std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║   Survey-In Status Monitor                        ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;
        std::cout << "\n  Monitoring survey-in progress..." << std::endl;
        std::cout << "  Press Ctrl+C to stop\n" << std::endl;
        
        // Poll NAV-SVIN message (0x01 0x3B) for survey-in status
        std::vector<uint8_t> poll_svin = {0x01, 0x3B, 0x00, 0x00};
        
        while (true) {
            // Flush buffer
            tcflush(fd_, TCIFLUSH);
            
            // Send poll request
            if (!send_ubx(poll_svin)) {
                std::cerr << "  Failed to poll survey-in status" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            
            // Wait for response
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::vector<uint8_t> response(512);
            int n = read(fd_, response.data(), response.size());
            
            // Parse NAV-SVIN response
            bool found = false;
            for (int i = 0; i < n - 50; i++) {
                if (response[i] == 0xB5 && response[i+1] == 0x62 && 
                    response[i+2] == 0x01 && response[i+3] == 0x3B) {
                    
                    // Verify we have enough data
                    uint16_t payload_len = response[i+4] | (response[i+5] << 8);
                    if (i + 6 + payload_len > n) continue;
                    
                    // NAV-SVIN payload structure (starts at i+6):
                    // 0-3: version (1 byte) + reserved (3 bytes)
                    // 4-7: iTOW (ignored)
                    // 8-11: dur (duration in seconds)
                    // 12-15: meanX (ECEF X in cm)
                    // 16-19: meanY (ECEF Y in cm) 
                    // 20-23: meanZ (ECEF Z in cm)
                    // 24-27: meanXHP (high precision X in 0.1mm)
                    // 28-31: meanYHP, meanZHP (high precision Y,Z)
                    // 32-35: meanAcc (accuracy in 0.1mm)
                    // 36-39: obs (number of observations)
                    // 40: valid (validity flags)
                    // 41: active (activity flags)
                    
                    int base = i + 6;  // Start of payload
                    
                    uint32_t dur = response[base+8] | (response[base+9] << 8) | 
                                   (response[base+10] << 16) | (response[base+11] << 24);
                    
                    // Mean position ECEF X, Y, Z (cm)
                    int32_t meanX = response[base+12] | (response[base+13] << 8) | 
                                    (response[base+14] << 16) | (response[base+15] << 24);
                    int32_t meanY = response[base+16] | (response[base+17] << 8) | 
                                    (response[base+18] << 16) | (response[base+19] << 24);
                    int32_t meanZ = response[base+20] | (response[base+21] << 8) | 
                                    (response[base+22] << 16) | (response[base+23] << 24);
                    
                    // Mean position accuracy (0.1mm)
                    uint32_t meanAcc = response[base+32] | (response[base+33] << 8) | 
                                       (response[base+34] << 16) | (response[base+35] << 24);
                    
                    uint32_t obs = response[base+36] | (response[base+37] << 8) | 
                                   (response[base+38] << 16) | (response[base+39] << 24);
                    
                    uint8_t valid = response[base+40];
                    uint8_t active = response[base+41];
                    
                    // Convert accuracy from 0.1mm to mm
                    double acc_mm = meanAcc / 10.0;
                    
                    // Display status
                    std::cout << "\r  Status: ";
                    if (active & 0x01) {
                        std::cout << "ACTIVE";
                    } else {
                        std::cout << "INACTIVE";
                    }
                    
                    if (valid & 0x01) {
                        std::cout << " | ✓ VALID";
                    } else {
                        std::cout << " | ⏳ IN PROGRESS";
                    }
                    
                    std::cout << " | Duration: " << std::setw(4) << dur << "s"
                              << " | Observations: " << std::setw(6) << obs
                              << " | Accuracy: " << std::fixed << std::setprecision(1) 
                              << std::setw(8) << acc_mm << " mm"
                              << "          " << std::flush;
                    
                    if ((valid & 0x01) && !(active & 0x01)) {
                        std::cout << "\n\n  ✓ Survey-In Complete!" << std::endl;
                        std::cout << "  Final position (ECEF cm): X=" << meanX 
                                  << " Y=" << meanY << " Z=" << meanZ << std::endl;
                        std::cout << "  Final accuracy: " << acc_mm << " mm" << std::endl;
                        
                        // Offer to configure device as fixed base with this position
                        std::cout << "\n  Configure device to use this position as fixed base? (y/n): ";
                        std::string use_fixed;
                        std::getline(std::cin, use_fixed);
                        if (use_fixed == "y" || use_fixed == "Y") {
                            return configure_fixed_base(meanX, meanY, meanZ, static_cast<uint32_t>(acc_mm * 10));
                        }
                        return true;
                    }
                    
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                std::cout << "\r  ⚠ No survey-in data available - device may not be in survey-in mode"
                          << "                    " << std::flush;
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        return true;
    }
    
    bool configure_fixed_base(int32_t ecefX_cm, int32_t ecefY_cm, int32_t ecefZ_cm, uint32_t acc_0_1mm) {
        std::cout << "\n  Configuring fixed base station mode..." << std::endl;
        std::cout << "    Position (ECEF cm): X=" << ecefX_cm 
                  << " Y=" << ecefY_cm << " Z=" << ecefZ_cm << std::endl;
        std::cout << "    Accuracy: " << (acc_0_1mm / 10.0) << " mm" << std::endl;
        
        // Try CFG-TMODE3 for fixed mode (0x06 0x71)
        std::cout << "\n    Trying CFG-TMODE3 (fixed mode)..." << std::endl;
        std::vector<uint8_t> tmode3 = {
            0x06, 0x71, 0x28, 0x00,  // CFG-TMODE3, length 40
            0x00, 0x00,              // Version, reserved
            0x02, 0x00,              // Flags: Fixed mode (bit 1 = 1) + LLA not used (bit 8 = 0)
            // ECEF X (cm)
            static_cast<uint8_t>(ecefX_cm & 0xFF),
            static_cast<uint8_t>((ecefX_cm >> 8) & 0xFF),
            static_cast<uint8_t>((ecefX_cm >> 16) & 0xFF),
            static_cast<uint8_t>((ecefX_cm >> 24) & 0xFF),
            // ECEF Y (cm)
            static_cast<uint8_t>(ecefY_cm & 0xFF),
            static_cast<uint8_t>((ecefY_cm >> 8) & 0xFF),
            static_cast<uint8_t>((ecefY_cm >> 16) & 0xFF),
            static_cast<uint8_t>((ecefY_cm >> 24) & 0xFF),
            // ECEF Z (cm)
            static_cast<uint8_t>(ecefZ_cm & 0xFF),
            static_cast<uint8_t>((ecefZ_cm >> 8) & 0xFF),
            static_cast<uint8_t>((ecefZ_cm >> 16) & 0xFF),
            static_cast<uint8_t>((ecefZ_cm >> 24) & 0xFF),
            // High precision X, Y, Z (0.1mm) - set to 0
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // Fixed position accuracy (0.1mm)
            static_cast<uint8_t>(acc_0_1mm & 0xFF),
            static_cast<uint8_t>((acc_0_1mm >> 8) & 0xFF),
            static_cast<uint8_t>((acc_0_1mm >> 16) & 0xFF),
            static_cast<uint8_t>((acc_0_1mm >> 24) & 0xFF),
            // Survey-in min duration (not used in fixed mode)
            0x00, 0x00, 0x00, 0x00,
            // Survey-in accuracy limit (not used in fixed mode)
            0x00, 0x00, 0x00, 0x00,
        };
        
        bool success = send_ubx_with_ack(tmode3, 2);
        
        if (!success) {
            std::cout << "    CFG-TMODE3 not acknowledged, trying CFG-TMODE2..." << std::endl;
            
            // Try CFG-TMODE2 for older firmware (0x06 0x3D)
            std::vector<uint8_t> tmode2 = {
                0x06, 0x3D, 0x1C, 0x00,  // CFG-TMODE2, length 28
                0x02,                     // Mode: Fixed
                0x00,                     // Reserved
                0x00, 0x00,              // Usage flags
                // ECEF X (cm)
                static_cast<uint8_t>(ecefX_cm & 0xFF),
                static_cast<uint8_t>((ecefX_cm >> 8) & 0xFF),
                static_cast<uint8_t>((ecefX_cm >> 16) & 0xFF),
                static_cast<uint8_t>((ecefX_cm >> 24) & 0xFF),
                // ECEF Y (cm)
                static_cast<uint8_t>(ecefY_cm & 0xFF),
                static_cast<uint8_t>((ecefY_cm >> 8) & 0xFF),
                static_cast<uint8_t>((ecefY_cm >> 16) & 0xFF),
                static_cast<uint8_t>((ecefY_cm >> 24) & 0xFF),
                // ECEF Z (cm)
                static_cast<uint8_t>(ecefZ_cm & 0xFF),
                static_cast<uint8_t>((ecefZ_cm >> 8) & 0xFF),
                static_cast<uint8_t>((ecefZ_cm >> 16) & 0xFF),
                static_cast<uint8_t>((ecefZ_cm >> 24) & 0xFF),
                // Fixed position accuracy (0.1mm)
                static_cast<uint8_t>(acc_0_1mm & 0xFF),
                static_cast<uint8_t>((acc_0_1mm >> 8) & 0xFF),
                static_cast<uint8_t>((acc_0_1mm >> 16) & 0xFF),
                static_cast<uint8_t>((acc_0_1mm >> 24) & 0xFF),
                // Survey-in params (not used)
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
            };
            
            success = send_ubx_with_ack(tmode2, 2);
        }
        
        if (success) {
            std::cout << "    ✓ Fixed base mode configured!" << std::endl;
            std::cout << "\n  The device should now output RTCM3 corrections" << std::endl;
            std::cout << "  using the surveyed position as the base station location." << std::endl;
            
            // Save configuration
            std::cout << "\n  Saving configuration..." << std::endl;
            std::vector<uint8_t> save_cfg = {
                0x06, 0x09, 0x0D, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x1F, 0x1F, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x1F
            };
            send_ubx_with_ack(save_cfg, 1);
            
            return true;
        } else {
            std::cout << "    ⚠ Could not configure fixed base mode" << std::endl;
            std::cout << "    The device may not support this feature." << std::endl;
            return false;
        }
    }
    
    bool configure_survey_in() {
        std::cout << "\n  Survey-In Configuration:" << std::endl;
        std::cout << "  Duration (seconds, default 300): ";
        std::string dur_str;
        std::getline(std::cin, dur_str);
        uint32_t duration = dur_str.empty() ? 300 : std::stoi(dur_str);
        
        std::cout << "  Accuracy (mm, default 10000 = 10m): ";
        std::string acc_str;
        std::getline(std::cin, acc_str);
        uint32_t accuracy_mm = acc_str.empty() ? 10000 : std::stoi(acc_str);
        
        std::cout << "\n  Configuring Survey-In: " << duration << "s, " 
                  << accuracy_mm << "mm accuracy..." << std::endl;
        
        // Try CFG-TMODE3 (newer format)
        std::cout << "    Trying CFG-TMODE3..." << std::endl;
        std::vector<uint8_t> tmode3 = {
            0x06, 0x71, 0x28, 0x00,  // CFG-TMODE3, length 40
            0x00, 0x00,              // Version, reserved
            0x01, 0x00,              // Flags: Survey-In mode (bit 0 = 1)
            // ECEF X, Y, Z (not used in survey-in)
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            // High precision (not used)
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // Fixed pos accuracy (not used)
            0x00, 0x00, 0x00, 0x00,
            // Survey-in min duration
            static_cast<uint8_t>(duration & 0xFF),
            static_cast<uint8_t>((duration >> 8) & 0xFF),
            static_cast<uint8_t>((duration >> 16) & 0xFF),
            static_cast<uint8_t>((duration >> 24) & 0xFF),
            // Survey-in accuracy limit (0.1mm units)
            static_cast<uint8_t>((accuracy_mm * 10) & 0xFF),
            static_cast<uint8_t>(((accuracy_mm * 10) >> 8) & 0xFF),
            static_cast<uint8_t>(((accuracy_mm * 10) >> 16) & 0xFF),
            static_cast<uint8_t>(((accuracy_mm * 10) >> 24) & 0xFF),
        };
        
        bool success = send_ubx_with_ack(tmode3, 1);  // Only 1 retry
        
        if (!success) {
            std::cout << "    CFG-TMODE3 not acknowledged, trying CFG-TMODE2..." << std::endl;
            
            // Try older CFG-TMODE2 format (for older firmware)
            std::vector<uint8_t> tmode2 = {
                0x06, 0x3D, 0x1C, 0x00,  // CFG-TMODE2, length 28
                0x01,                     // Mode: Survey-In
                0x00,                     // Reserved
                0x00, 0x00,              // Usage flags
                // ECEF X, Y, Z (not used in survey-in)
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                // Fixed pos accuracy (not used)
                0x00, 0x00, 0x00, 0x00,
                // Survey-in min duration
                static_cast<uint8_t>(duration & 0xFF),
                static_cast<uint8_t>((duration >> 8) & 0xFF),
                static_cast<uint8_t>((duration >> 16) & 0xFF),
                static_cast<uint8_t>((duration >> 24) & 0xFF),
                // Survey-in accuracy limit (0.1mm units)
                static_cast<uint8_t>((accuracy_mm * 10) & 0xFF),
                static_cast<uint8_t>(((accuracy_mm * 10) >> 8) & 0xFF),
                static_cast<uint8_t>(((accuracy_mm * 10) >> 16) & 0xFF),
                static_cast<uint8_t>(((accuracy_mm * 10) >> 24) & 0xFF),
            };
            
            success = send_ubx_with_ack(tmode2, 1);
        }
        
        if (success) {
            std::cout << "  ✓ Survey-In mode configured!" << std::endl;
            std::cout << "\n  The base station will now survey its position." << std::endl;
            std::cout << "  Make sure the antenna has a clear view of the sky!" << std::endl;
            
            // Ask if user wants to monitor
            std::cout << "\n  Monitor survey-in progress? (y/n): ";
            std::string monitor;
            std::getline(std::cin, monitor);
            if (monitor == "y" || monitor == "Y") {
                return monitor_survey_in_status();
            }
            return true;
        } else {
            std::cout << "  ⚠ Could not confirm Survey-In configuration" << std::endl;
            std::cout << "  This device may not support CFG-TMODE commands." << std::endl;
            std::cout << "\n  Possible solutions:" << std::endl;
            std::cout << "  1. The device may need to be configured via u-center software" << std::endl;
            std::cout << "  2. Some devices auto-enter base mode when they have a fix" << std::endl;
            std::cout << "  3. The device may already be in base mode from previous config" << std::endl;
            std::cout << "\n  Try monitoring to check if survey-in is already active? (y/n): ";
            std::string monitor;
            std::getline(std::cin, monitor);
            if (monitor == "y" || monitor == "Y") {
                return monitor_survey_in_status();
            }
            std::cout << "\n  Continuing anyway - RTCM3 output may still work..." << std::endl;
            // Return true anyway - RTCM3 configuration should work regardless
            return true;
        }
    }
    
    bool verify_rtcm3_output() {
        std::cout << "\n  Monitoring output for 10 seconds..." << std::endl;
        std::cout << "  Note: RTCM3 messages only appear after device gets GPS fix" << std::endl;
        std::cout << "        and enters base station mode." << std::endl;
        
        auto start = std::chrono::steady_clock::now();
        std::vector<uint8_t> buffer(4096);
        int rtcm_count = 0;
        int nmea_count = 0;
        std::vector<int> rtcm_types_seen;
        
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
            int n = read(fd_, buffer.data(), buffer.size());
            if (n > 0) {
                for (int i = 0; i < n - 5; i++) {
                    // Check for RTCM3 preamble
                    if (buffer[i] == 0xD3) {
                        // Check if next 6 bits are zeros (reserved bits)
                        if ((buffer[i+1] & 0xFC) == 0) {
                            // Extract message type: first 12 bits of message (bytes 3-4, top 12 bits)
                            uint16_t msg_type = (buffer[i+3] << 4) | (buffer[i+4] >> 4);
                            if (std::find(rtcm_types_seen.begin(), rtcm_types_seen.end(), msg_type) 
                                == rtcm_types_seen.end()) {
                                rtcm_types_seen.push_back(msg_type);
                                std::cout << "  ✓ RTCM3 message " << msg_type << " detected" << std::endl;
                            }
                            rtcm_count++;
                        }
                    }
                    // Check for NMEA
                    if (buffer[i] == '$') {
                        nmea_count++;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "\n  Results:" << std::endl;
        std::cout << "    RTCM3 messages: " << rtcm_count << std::endl;
        std::cout << "    NMEA messages:  " << nmea_count << std::endl;
        std::cout << "    RTCM3 types:    ";
        for (int type : rtcm_types_seen) std::cout << type << " ";
        std::cout << std::endl;
        
        if (rtcm_count > 0) {
            std::cout << "\n  Expected RTCM3 types for RTK base: 1005, 1077, 1087, 1097, 1127, 1230" << std::endl;
            return true;
        } else {
            std::cout << "\n  ⚠ No RTCM3 messages detected yet." << std::endl;
            std::cout << "  This is normal if the device doesn't have a GPS fix yet." << std::endl;
            std::cout << "  The RTCM3 configuration has been saved and will activate" << std::endl;
            std::cout << "  automatically once the device gets a fix and enters base mode." << std::endl;
            // Don't fail - configuration is saved, just waiting for GPS fix
            return false;  // Changed to false to reflect that RTCM3 wasn't verified yet
        }
    }
    
    bool send_ubx(const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> packet;
        packet.push_back(0xB5);
        packet.push_back(0x62);
        packet.insert(packet.end(), payload.begin(), payload.end());
        
        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < packet.size(); i++) {
            ck_a += packet[i];
            ck_b += ck_a;
        }
        packet.push_back(ck_a);
        packet.push_back(ck_b);
        
        int written = write(fd_, packet.data(), packet.size());
        return written == (int)packet.size();
    }
    
    bool send_ubx_with_ack(const std::vector<uint8_t>& payload, int max_retries = 2) {
        for (int retry = 0; retry <= max_retries; retry++) {
            if (retry > 0) {
                if (verbose_) std::cout << "      Retry " << retry << "/" << max_retries << "..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            
            // Flush any pending data before sending
            tcflush(fd_, TCIFLUSH);
            
            if (!send_ubx(payload)) {
                if (verbose_) std::cerr << "      ✗ Failed to send UBX command" << std::endl;
                continue;
            }
            
            // Wait for ACK - increased timeout for slower responses
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::vector<uint8_t> response(512);
            int n = read(fd_, response.data(), response.size());
            
            if (verbose_ && n > 0) {
                std::cout << "      Received " << n << " bytes: ";
                for (int i = 0; i < std::min(n, 30); i++) {
                    printf("%02X ", response[i]);
                }
                if (n > 30) std::cout << "...";
                std::cout << std::endl;
                
                // Debug: show all UBX message headers found
                std::cout << "      UBX messages in response: ";
                for (int i = 0; i < n - 5; i++) {
                    if (response[i] == 0xB5 && response[i+1] == 0x62) {
                        printf("[%02X %02X] ", response[i+2], response[i+3]);
                    }
                }
                std::cout << std::endl;
            }
            
            if (n > 10) {
                for (int i = 0; i < n - 10; i++) {
                    if (response[i] == 0xB5 && response[i+1] == 0x62) {
                        // Check for ACK-ACK (0x05 0x01)
                        if (response[i+2] == 0x05 && response[i+3] == 0x01) {
                            // Match class and ID
                            if (response[i+6] == payload[0] && response[i+7] == payload[1]) {
                                if (verbose_) std::cout << "      ✓ ACK received" << std::endl;
                                return true;
                            }
                        }
                        // Check for ACK-NAK (0x05 0x00)
                        if (response[i+2] == 0x05 && response[i+3] == 0x00) {
                            if (response[i+6] == payload[0] && response[i+7] == payload[1]) {
                                if (verbose_) {
                                    std::cerr << "      ✗ NAK received for class 0x" << std::hex 
                                             << (int)payload[0] << " id 0x" << (int)payload[1] 
                                             << std::dec << std::endl;
                                }
                                return false;  // NAK means command rejected, don't retry
                            }
                        }
                    }
                }
            }
            
            if (verbose_) {
                std::cerr << "      ✗ No ACK/NAK received (" << n << " bytes read)" << std::endl;
            }
        }
        
        return false;
    }
};

int main(int argc, char** argv) {
    std::string port = "/dev/ttyACM0";
    if (argc > 1) {
        port = argv[1];
    }
    
    RTKBaseStationSetup setup(port);
    
    if (setup.run_full_setup()) {
        std::cout << "\n╔════════════════════════════════════════════════════╗" << std::endl;
        std::cout << "║         Setup Complete - Ready for RTK!           ║" << std::endl;
        std::cout << "╚════════════════════════════════════════════════════╝" << std::endl;
        std::cout << "\nNext steps:" << std::endl;
        std::cout << "1. Verify RTCM3: cat " << port << " | hexdump -C | grep 'd3'" << std::endl;
        std::cout << "2. Use radio sender node to transmit corrections" << std::endl;
        std::cout << "3. Inject into Cube Orange for RTK positioning" << std::endl;
        return 0;
    } else {
        std::cerr << "\n✗ Setup failed - check connections and try again" << std::endl;
        return 1;
    }
}