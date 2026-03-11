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
//#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node{
public:
    SerialBridgeNode() : Node("serial_bridge_node"){
        publisher_.push_back(this->create_publisher<std_msgs::msg::Int32>("distance0", 10));
        publisher_.push_back(this->create_publisher<std_msgs::msg::Int32>("distance1", 10));

        //subscribe to scan topic
        //scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),std::bind(&SerialBridgeNode::scan_callback, this, std::placeholders::_1));

        // Start Outputting
        RCLCPP_INFO(this->get_logger(), "=============================================");
        RCLCPP_INFO(this->get_logger(), "UART Bridge Started");
        RCLCPP_INFO(this->get_logger(), "=============================================");
        
        // open connection to linux serial device, should be ttyUSB4 if only one UART bus is activated.
        // for multiple UART nodes, we'll publish to multiple topics. 
        
        serial_port_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

        if(serial_port_ < 0){
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        if(tcgetattr(serial_port_, &tty) != 0){
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
        read_timer_ = this->create_wall_timer(10ms, std::bind(&SerialBridgeNode::read_serial, this));
        //write_timer_ = this->create_wall_timer(200ms, std::bind(&SerialBridgeNode::write_serial, this));
    }

    ~SerialBridgeNode(){ 
        if (serial_port_ >= 0){
        close(serial_port_);
        RCLCPP_INFO(this->get_logger(), "Serial Port Closed");
        }
    }

private:
    void read_serial(){
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

                // Check for Dist1
                if(line.find("Dist1") != std::string::npos){
                    try{
                        int value = std::stoi(line.substr(line.find(":") + 1));
                        std_msgs::msg::Int32 msg;
                        msg.data = value;
                        publisher_[0]->publish(msg);
                    }
                    catch(...){
                        RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                    }
                }

                // Check for Dist2
                else if(line.find("Dist2") != std::string::npos){
                    try{
                        int value = std::stoi(line.substr(line.find(":") + 1));
                        std_msgs::msg::Int32 msg;
                        msg.data = value;
                        publisher_[1]->publish(msg);
                    }
                    catch(...){
                        RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                    }
                }
            }
        }
    }
    /*
    void write_serial(){
        if(scan_data_min_ >= 0.0){
            std::stringstream ss;
            ss << "[TOPIC] LiDAR:" << std::fixed << std::setprecision(2) << scan_data_min_ << "\n";
            std::string data = ss.str();

            if(serial_port_ >= 0){
                ssize_t byte_write = write(serial_port_, data.c_str(), data.size());
                if(byte_write < 0 && errno != EAGAIN){
                    RCLCPP_ERROR(this->get_logger(), "Error writing: %s", strerror(errno));
                }
            }
        }
    }
    float scan_data_min_ = -1.0;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        float min_dist = msg->range_max;
        for(auto range : msg->ranges){
            if(range < min_dist && range > msg->range_min){
                min_dist = range;
            }
        }
        scan_data_min_ = min_dist;
    };
    */
    std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> publisher_;
    //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    //rclcpp::TimerBase::SharedPtr write_timer_;
    int serial_port_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}