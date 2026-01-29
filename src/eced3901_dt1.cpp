/*
Code for DT1
V. Sieben
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>
#include <algorithm> 

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"
 
// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// PID logic
#include "integrator.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
  public:
	// Constructor creates a node named Square_Routine. 
	SquareRoutine() : Node("Square_Routine")
	{
		// Create the subscription
		// The callback function executes whenever data is published to the 'topic' topic.
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
          
		// Create the publisher
		// Publisher to a topic named "topic". The size of the queue is 10 messages.
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      
	  	// Create the timer
	  	timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
	}

  private:
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;
		
		q_x = msg->pose.pose.orientation.x;	
		q_y = msg->pose.pose.orientation.y;	
		q_z = msg->pose.pose.orientation.z;	
		q_w = msg->pose.pose.orientation.w;		
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired.");
	}
	
	void timer_callback()
	{
		geometry_msgs::msg::Twist msg;
		tf2::Quaternion q(q_x, q_y, q_z, q_w);	// Quaternion	
        	tf2::Matrix3x3 m(q);			// 3x3 Rotation matrix from quaternion
        	double roll, pitch, yaw;
        	m.getRPY(roll, pitch, yaw);  		// Roll Pitch Yaw from rotation matrix

        	
		// Calculate distance travelled from initial
		d_now =	pow( pow(x_now - x_init, 2) + pow(y_now - y_init, 2), 0.5 );
		
		// Calculate angle travelled from initial
		th_now = yaw;
	    double yaw_err = wrap_angle(th_target - th_now);

        // distance remaining
        double d_err = d_aim - d_now;
        
        if (debug_ticks_ < 10) {
            RCLCPP_INFO(
                this->get_logger(),
                "DEBUG [%d] th_now=%.4f th_target=%.4f yaw_err=%.4f w_cmd=%.4f",
                debug_ticks_, th_now, th_target, yaw_err, msg.angular.z
            );
            debug_ticks_++;
        }


        if (d_err > d_tol)
        {
            uint32_t t = now_ms();
            
            // Use PID to drive distance error to 0
            // setpoint = 0, measurement = -d_err -> error = d_err
            float v_cmd = pid_step_ms(&pid_lin, 0.0f, (float)(-d_err), t);
            
            // yaw-hold PID
            float w_cmd = pid_step_ms(&pid_yaw, 0.0f, -(float)yaw_err, t);

            if (debug_ticks_ < 10) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "DEBUG move start [%d] th_now=%.4f th_target=%.4f yaw_err=%.4f w_cmd=%.4f",
                    debug_ticks_,
                    th_now,
                    th_target,
                    yaw_err,
                    msg.angular.z
                );
                debug_ticks_++;
            }

            msg.linear.x = (double)v_cmd;
            msg.angular.z = (double)w_cmd;

            publisher_->publish(msg);
        }	
        
        // Keep turning if not reached last angular target		
		// If done step, stop
        else if (std::abs(yaw_err) > th_tol)
        {
            uint32_t t = now_ms();

            // error = wrap_angle(th_target - th_now)
            // Drive error -> 0 with PID by using setpoint=0, measurement=error
            float w_cmd = pid_step_ms(&pid_yaw, 0.0f, -(float)yaw_err, t);

            msg.linear.x = 0.0;
            msg.angular.z = (double)w_cmd;
            publisher_->publish(msg);
        } 

        else
		{
			msg.linear.x = 0; //double(rand())/double(RAND_MAX); //fun
			msg.angular.z = 0; //2*double(rand())/double(RAND_MAX) - 1; //fun
			publisher_->publish(msg);
			last_state_complete += 1;
		}

		sequence_statemachine();		
		

		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	}
	
	void sequence_statemachine()
	{
		if (last_state_complete == 10) // force multiple, (acting as a "stop state")
		{
			switch(count_) 
			{
			  case 0:
			    move_distance(1); //move the robot forwards
			    break;
			  case 1:
			    turn_angle(M_PI/2);  //turn the robot
			    break;
			  case 2:
			    move_distance(1);  //second movement for the robot
			    break;
			  case 3:
			    turn_angle(M_PI/2); //turn the robot again
			    break;
			  case 4:
			    move_distance(1);  //third movement for the robot
			    break;			    
			  case 5:
			    turn_angle(M_PI/2);  //final turn
			    break;
			  case 6:
			    move_distance(1); //last movement for the robot
			    break;			    
			  case 7:
			    turn_angle(M_PI/2);  //re-orient the robot
			    break;  
			  default:
			    break;
			}
		}			
	}
	
	// Set the initial position as where robot is now and put new d_aim in place	
	void move_distance(double distance)
	{
        debug_ticks_ = 0;

		d_aim = distance;
		x_init = x_now;
		y_init = y_now;		

        th_target = th_now;

        uint32_t t = now_ms();

        pid_init(&pid_lin, 
                1.0,           // kp 
                1.0f,           // ki
                0.00f,          // kd
                -(float)v_max, 
                (float)v_max, 
                -0.5f, 
                0.5f, 
                t);

        // reset yaw PID for heading hold
        pid_init(&pid_yaw, 
                0.8f, 
                0.0f, 
                0.0f,          
                -(float)w_max, 
                (float)w_max,
                -0.2f,
                0.2f, 
                t);

		count_++;		// advance state counter
		last_state_complete = 0;	
	}
	
	// Set the initial angle as where robot is heading and put new th_aim in place			
	void turn_angle(double angle)
	{
        th_init = th_now;
        th_target = wrap_angle(th_init + angle);

        uint32_t t = now_ms();
        pid_init(&pid_yaw,
                0.5f,           // kp
                0.00f,           // ki
                0.00f,          // kd
                -(float)w_max,
                (float)w_max,
                -0.5f,
                0.5f,
                t);

        count_++;
        last_state_complete = 0;
	}
	
	// Handle angle wrapping
    double wrap_angle(double angle)
    {
        angle = std::remainder(angle, 2.0 * M_PI);
        return angle;
    }

    int debug_ticks_ = 0;
	
	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double x_vel = 0.1, th_vel = 0.2;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0, th_now = 0, th_init = 0;
	double d_now = 0, d_aim = 0, th_aim = 0;
	double q_x = 0, q_y = 0, q_z = 0, q_w = 0; 
    double th_target = 0.0;
	size_t count_ = 0;
	int last_state_complete = 1;

    // Custom PID controller
    PID pid_lin{};
    PID pid_yaw{};

    // Tuning + tolerances
    double d_tol = 0.01;     // meters
    double th_tol = 0.01;    // radians 

    // Output limits
    double v_max = 0.30;     // m/s
    double w_max = 0.20;     // rad/s 

    // Helper to get ms time for PID
    uint32_t now_ms()
    {
        return (uint32_t)(this->get_clock()->now().nanoseconds() / 1000000ULL);
    }
};


    	


//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char * argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);
  
	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());
 
	// Stop node 
	rclcpp::shutdown();
	return 0;
}

