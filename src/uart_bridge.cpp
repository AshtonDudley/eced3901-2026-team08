// Author: Alexandre DesAulniers
// Last Updated, Feb 11th 2026 @ 11:08AM
// Edits: Liam Legge

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node {
public:
    SerialBridgeNode() : Node("serial_bridge_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("ultrasonic_distance", 10);
        
        // open connection to linux serial device, should be ttyUSB4 if only one UART bus is activated.
        // for multiple UART nodes, we'll publish to multiple topics. 
        
        serial_port_ = open("/dev/ttyUSB4", O_RDWR | O_NOCTTY | O_NONBLOCK);

        // set 115200 baud for Serial Port
        struct termios tty;
        if(tcgetattr(serial_port_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;                 /* 8-bit characters */
        tty.c_cflag &= ~PARENB;             /* no parity bit */
        tty.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */
        tty.c_lflag &= ~ICANON; // Disable canonical mode
        tty.c_lflag &= ~ECHO;   // Disable echo
        tty.c_lflag &= ~ECHOE;  // Disable erasure
        tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INPCK|IGNCR|ICRNL); // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline translation)

        tcsetattr(serial_port_, TCSANOW, &tty);

        // Ceate timer loop to read serial
        timer_ = this->create_wall_timer(10ms, std::bind(&SerialBridgeNode::read_serial, this));
    }

    ~SerialBridgeNode() { close(serial_port_); }

private:
    void read_serial() {
        char buf[16];
        int n = read(serial_port_, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std:string input(buf);
            std:string prefix = "[TOPIC] minDistance";
            size_t pos = input.find(prefix);
            if (pos != std::string::npos) {
                size_t start = pos + prefix.length();
                size_t end = input.find_first_not_of("0123456789", start);
                std::string number_str = input.substr(start, end - start);
                try {
                    auto message = std_msgs::msg::Int32();
                    message.data = std::stoi(number_str);
                    publisher_->publish(message);
            } catch (...) {
                // Ignore partial lines or other non integer shit
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeNode>());
    rclcpp::shutdown();
    return 0;
}