// Author: Alexandre DesAulniers
// Last Updated, Feb 11th 2026 @ 11:08AM
// Edits: Liam Legge

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <errno.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode() : Node("serial_bridge_node") {
        publisher_.push_back(this->create_publisher<std_msgs::msg::Int32>("distance0", 10));
        publisher_.push_back(this->create_publisher<std_msgs::msg::Int32>("distance1", 10));

        // Start Outputting
        RCLCPP_INFO(this->get_logger(), "=============================================");
        RCLCPP_INFO(this->get_logger(), "UART Bridge Started");
        RCLCPP_INFO(this->get_logger(), "=============================================");
        
        // open connection to linux serial device, should be ttyUSB4 if only one UART bus is activated.
        // for multiple UART nodes, we'll publish to multiple topics. 
        
        serial_port_ = open("/dev/ttyUSB4", O_RDWR | O_NOCTTY | O_NONBLOCK);

        if(serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        if(tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
        }

        // set 115200 baud for Serial Port
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // Read Flags
        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;                 /* 8-bit characters */
        tty.c_cflag &= ~PARENB;             /* no parity bit */
        tty.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;


        // Check and Flush Bugger
        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting termios: %s", strerror(errno));
        }
        tcflush(serial_port_, TCIFLUSH);

        // Ceate timer loop to read serial
        timer_ = this->create_wall_timer(10ms, std::bind(&SerialBridgeNode::read_serial, this));
    }

    ~SerialBridgeNode() { 
        if (serial_port_ >= 0){
        close(serial_port_);
        RCLCPP_INFO(this->get_logger(), "Serial Port Closed");
        }
    }

private:
    void read_serial() {
        static std::string buffer;
        char buf[64];
        int n = read(serial_port_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            buffer.append(buf);

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos){
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                if(line.empty()) continue; 
                if(!line.empty() && line[0] == ','){
                    line = "0" + line;
                }

                std::vector<int> distances;
                std::stringstream ss(line);
                std::string token;

                while(std::getline(ss, token, ',')){
                    if(token.empty()){
                        distances.push_back(0);
                    } else {
                        try {
                            distances.push_back(std::stoi(token));
                        } catch (...) {
                            distances.push_back(0);
                        }
                    }
                }

                if(distances.size() == publisher_.size()){
                    for(size_t i = 0; i < publisher_.size(); ++i){
                        std_msgs::msg::Int32 msg;
                        msg.data = distances[i];
                        publisher_[i]->publish(msg);
                    }
                }
            }
        }
    }

    std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}