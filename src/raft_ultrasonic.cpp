/*
Code for DT1
V. Sieben
Version 3.0
License: GNU GPLv3

Motion sequence:
  1. APPROACH  — drive forward until ultrasonic reads <= 0.2m
  2. SCOOP     — turn 90° CCW, then 90° CW
  3. RETURN    — turn 180° CW, drive back the same distance to start
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/range.hpp"        // Published by ultrasonic_serial_node

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

enum class Mode { APPROACH, SCOOP, RETURN, DONE };


class MotionSequence : public rclcpp::Node
{
public:
    MotionSequence() : Node("Motion_Sequence")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MotionSequence::odom_callback, this, _1));

        // Matches topic published by ultrasonic_serial_node.cpp
        ultrasonic_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ultrasonic/range", 10, std::bind(&MotionSequence::ultrasonic_callback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MotionSequence::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node started — approaching object...");
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
    // Ultrasonic callback — just stores the latest reading
    // ---------------------------------------------------------------------------------
    void ultrasonic_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        if (std::isfinite(msg->range))
            ultrasonic_range_ = msg->range;
    }

    // ---------------------------------------------------------------------------------
    // Main control loop (10 Hz)
    // ---------------------------------------------------------------------------------
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd;

        d_now = sqrt(pow(x_now - x_init, 2) + pow(y_now - y_init, 2));

        switch (mode_)
        {
            case Mode::APPROACH: run_approach(cmd); break;
            case Mode::SCOOP:    run_scoop(cmd);    break;
            case Mode::RETURN:   run_return(cmd);   break;
            case Mode::DONE:     break;             // stay stopped
        }

        publisher_->publish(cmd);
    }

    // ---------------------------------------------------------------------------------
    // APPROACH — drive forward until within 0.2m of object
    // ---------------------------------------------------------------------------------
    void run_approach(geometry_msgs::msg::Twist &cmd)
    {
        // Snapshot start position on first tick
        if (!start_recorded_)
        {
            x_start_ = x_now;
            y_start_ = y_now;
            x_init   = x_now;
            y_init   = y_now;
            start_recorded_ = true;
            RCLCPP_INFO(this->get_logger(), "Start position recorded: (%.2f, %.2f)", x_start_, y_start_);
        }

        if (ultrasonic_range_ > SCOOP_TRIGGER_DIST)
        {
            // Keep driving forward
            cmd.linear.x = x_vel;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Approaching... range: %.2fm", ultrasonic_range_);
        }
        else
        {
            // Object reached — record distance travelled and begin scoop
            distance_travelled_ = sqrt(pow(x_now - x_start_, 2) + pow(y_now - y_start_, 2));
            RCLCPP_INFO(this->get_logger(),
                "Object reached at %.2fm. Distance travelled: %.2fm. Starting scoop.",
                ultrasonic_range_, distance_travelled_);

            scoop_step_         = 0;
            last_step_complete_ = true;
            mode_               = Mode::SCOOP;
        }
    }

    // ---------------------------------------------------------------------------------
    // SCOOP — CCW 90°, then CW 90°
    // ---------------------------------------------------------------------------------
    void run_scoop(geometry_msgs::msg::Twist &cmd)
    {
        if (last_step_complete_)
        {
            switch (scoop_step_)
            {
                case 0:
                    RCLCPP_INFO(this->get_logger(), "Scoop step 1: Turning 90° CCW");
                    turn_angle(M_PI / 2.05, +1);    // CCW
                    scoop_step_++;
                    break;
                case 1:
                    RCLCPP_INFO(this->get_logger(), "Scoop step 2: Turning 90° CW");
                    turn_angle(M_PI / 2.05, -1);    // CW
                    scoop_step_++;
                    break;
                case 2:
                    RCLCPP_INFO(this->get_logger(), "Scoop complete. Starting return.");
                    return_step_        = 0;
                    last_step_complete_ = true;
                    mode_               = Mode::RETURN;
                    return;
            }
        }

        execute_turn(cmd);
    }

    // ---------------------------------------------------------------------------------
    // RETURN — 180° CW turn, then drive back the same distance
    // ---------------------------------------------------------------------------------
    void run_return(geometry_msgs::msg::Twist &cmd)
    {
        if (last_step_complete_)
        {
            switch (return_step_)
            {
                case 0:
                    RCLCPP_INFO(this->get_logger(), "Return step 1: Turning 180° CW");
                    turn_angle(M_PI / 1.01, -1);    // CW 180°
                    return_step_++;
                    break;
                case 1:
                    RCLCPP_INFO(this->get_logger(), "Return step 2: Driving %.2fm back to start",
                        distance_travelled_);
                    move_distance(distance_travelled_);
                    return_step_++;
                    break;
                case 2:
                    RCLCPP_INFO(this->get_logger(), "Returned to start. Mission complete!");
                    mode_ = Mode::DONE;
                    return;
            }
        }

        // Execute current motion (turn or drive)
        if (is_turning_)
            execute_turn(cmd);
        else
            execute_drive(cmd);
    }

    // ---------------------------------------------------------------------------------
    // Motion primitives
    // ---------------------------------------------------------------------------------
    void turn_angle(double angle, int direction)
    {
        th_aim_             = angle;
        th_init_            = th_now;
        turn_dir_           = direction;
        is_turning_         = true;
        last_step_complete_ = false;
    }

    void move_distance(double distance)
    {
        d_aim_              = distance;
        x_init              = x_now;
        y_init              = y_now;
        is_turning_         = false;
        last_step_complete_ = false;
    }

    void execute_turn(geometry_msgs::msg::Twist &cmd)
    {
        if (fabs(wrap_angle(th_now - th_init_)) < th_aim_)
        {
            cmd.angular.z = turn_dir_ * th_vel;
        }
        else
        {
            last_step_complete_ = true;
        }
    }

    void execute_drive(geometry_msgs::msg::Twist &cmd)
    {
        if (d_now < d_aim_)
        {
            cmd.linear.x = x_vel;
        }
        else
        {
            last_step_complete_ = true;
        }
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---------------------------------------------------------------------------------
    // State variables
    // ---------------------------------------------------------------------------------
    double x_vel  = 0.1;    // forward speed (m/s)
    double th_vel = 0.2;    // turn speed (rad/s)

    double x_now  = 0, y_now  = 0, th_now  = 0;
    double x_init = 0, y_init = 0;
    double d_now  = 0, d_aim_ = 0, th_aim_ = 0, th_init_ = 0;
    double x_start_ = 0, y_start_ = 0;
    double distance_travelled_  = 0;
    double ultrasonic_range_    = 9999.0;

    int  turn_dir_    = 1;
    int  scoop_step_  = 0;
    int  return_step_ = 0;

    bool start_recorded_    = false;
    bool last_step_complete_ = true;
    bool is_turning_        = false;

    Mode mode_ = Mode::APPROACH;

    static constexpr double SCOOP_TRIGGER_DIST = 0.20;  // metres
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
