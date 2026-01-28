#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp" //odom library

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class OdomMeasure : public rclcpp::Node
{
 public:
    OdomMeasure() : Node("odom_measure"){
	sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
	    "/odom",10,
	    std::bind(&OdomMeasure::odomCallback,this,std::placeholders::_1)
	    );

        file_.open("yaw_log.csv", std::ios::out);
        file_ << "time_sec, yaw_rad\n";
    }

    ~OdomMeasure(){
	file_.close();
    }

 private:

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
	double qx = msg->pose.pose.orientation.x;
	double qy = msg->pose.pose.orientation.y;
	double qz = msg->pose.pose.orientation.z;
	double qw = msg->pose.pose.orientation.w;

	tf2::Quaternion q(qx,qy,qz,qw);
	tf2::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);

	double time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

	file_ << time_sec << "," << yaw << "\n";
    }

   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
   std::ofstream file_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomMeasure>());
    rclcpp::shutdown();
    return 0;
}
