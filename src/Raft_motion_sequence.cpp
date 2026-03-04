/*
Code for DT1
V. Sieben
Version 2.0
License: GNU GPLv3

Motion sequence:
  1. Drive forward 0.30m
  2. Turn 90° counter-clockwise
  3. Turn 90° clockwise
  4. Turn 180°
  5. Drive forward 0.30m (back to start)
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


class MotionSequence : public rclcpp::Node
{
public:
    MotionSequence() : Node("Motion_Sequence")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MotionSequence::odom_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MotionSequence::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Motion sequence node started.");
    }

private:

    // ---------------------------------------------------------------------------------
    // Odometry callback
    // ---------------------------------------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_now = msg->pose.pose.position.x;
        y_now = msg->pose.pose.position.y;

        double q_x = msg->pose.pose.orientation.x;
        double q_y = msg->pose.pose.orientation.y;
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;

        tf2::Quaternion q(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, th_now);
    }

    // ---------------------------------------------------------------------------------
    // Main control loop
    // ---------------------------------------------------------------------------------
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        d_now = sqrt(pow(x_now - x_init, 2) + pow(y_now - y_init, 2));

        if (done_)
        {
            // Sequence complete — stay stopped
            publisher_->publish(msg);
            return;
        }

        // Execute current motion step
        if (d_now < d_aim)
        {
            // Still driving forward
            msg.linear.x = x_vel;
        }
        else if (fabs(wrap_angle(th_now - th_init)) < th_aim)
        {
            // Still turning
            msg.linear.x = 0;
            msg.angular.z = turn_dir_ * th_vel;
        }
        else
        {
            // Step complete
            msg.linear.x = 0;
            msg.angular.z = 0;
            last_state_complete = true;
        }

        publisher_->publish(msg);
        sequence_statemachine();
    }

    // ---------------------------------------------------------------------------------
    // State machine
    //
    //  Step 0: Drive forward 0.30m
    //  Step 1: Turn 90° CCW  (+)
    //  Step 2: Turn 90° CW   (-)
    //  Step 3: Turn 180°     (CW, -)
    //  Step 4: Drive forward 0.30m  (back to start)
    //  Step 5: Done
    // ---------------------------------------------------------------------------------
    void sequence_statemachine()
    {
        if (!last_state_complete) return;

        switch (step_)
        {
            case 0:
                RCLCPP_INFO(this->get_logger(), "Step 1: Driving forward 30cm");
                move_distance(0.30);
                break;

            case 1:
                RCLCPP_INFO(this->get_logger(), "Step 2: Turning 90° CCW");
                turn_angle(M_PI / 2.05, +1);   // +1 = CCW
                break;

            case 2:
                RCLCPP_INFO(this->get_logger(), "Step 3: Turning 90° CW");
                turn_angle(M_PI / 2.05, -1);   // -1 = CW
                break;

            case 3:
                RCLCPP_INFO(this->get_logger(), "Step 4: Turning 180°");
                turn_angle(M_PI / 1.01, -1);   // -1 = CW
                break;

            case 4:
                RCLCPP_INFO(this->get_logger(), "Step 5: Driving forward 30cm (returning to start)");
                move_distance(0.30);
                break;

            case 5:
                RCLCPP_INFO(this->get_logger(), "Sequence complete. Stopped.");
                done_ = true;
                break;
        }
    }

    // ---------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------
    void move_distance(double distance)
    {
        d_aim  = distance;
        th_aim = 0.0;           // not turning
        x_init = x_now;
        y_init = y_now;
        step_++;
        last_state_complete = false;
    }

    void turn_angle(double angle, int direction)
    {
        th_aim   = angle;
        d_aim    = 0.0;         // not moving
        th_init  = th_now;
        turn_dir_ = direction;
        step_++;
        last_state_complete = false;
    }

    double wrap_angle(double angle)
    {
        angle = fmod(angle + M_PI, 2 * M_PI);
        if (angle <= 0.0) angle += 2 * M_PI;
        return angle - M_PI;
    }

    // ---------------------------------------------------------------------------------
    // ROS2 interfaces
    // ---------------------------------------------------------------------------------
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---------------------------------------------------------------------------------
    // State variables
    // ---------------------------------------------------------------------------------
    double x_vel  = 0.1;    // forward speed (m/s)
    double th_vel = 0.2;    // turn speed (rad/s)

    double x_now  = 0, y_now  = 0, th_now  = 0;
    double x_init = 0, y_init = 0, th_init = 0;
    double d_now  = 0, d_aim  = 0, th_aim  = 0;

    int    turn_dir_          = 1;      // +1 CCW, -1 CW
    int    step_              = 0;
    bool   last_state_complete = true;
    bool   done_              = false;
};


// ---------------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionSequence>());
    rclcpp::shutdown();
    return 0;
}
