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

using namespace std::chrono_literals;
using std::placeholders::_1;


// Returns a commanded speed based on remaining error.
// error: signed (target - current). Sign determines direction.
// v_max: max magnitude of speed (m/s or rad/s)
// v_min: minimum magnitude once you’re moving (helps overcome stiction). Set 0 if unwanted.
// slow_zone: error magnitude where ramping starts (units match error)
// tol: within this tolerance, return 0 (done)
static double ramp_speed_from_error(double error,
                                   double v_max,
                                   double v_min,
                                   double slow_zone,
                                   double tol)
{
    const double e = std::abs(error);

    if (e <= tol) return 0.0;

    // Ramp factor: 1 outside slow zone, linearly down to 0 at tol
    double ramp = 1.0;
    if (e < slow_zone) {
        // Map e in [tol, slow_zone] -> ramp in [0, 1]
        ramp = (e - tol) / (slow_zone - tol);
        ramp = std::clamp(ramp, 0.0, 1.0);
    }

    // Speed magnitude with clamp to [v_min, v_max]
    double mag = v_max * ramp;
    mag = std::clamp(mag, v_min, v_max);

    // Restore direction
    return (error >= 0.0) ? mag : -mag;
}

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
	  	timer_ = this->create_wall_timer(20ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
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
		
	    double error = wrap_angle(th_target - th_now);
        
		// Keep moving if not reached last distance target
		if (d_now < d_aim)
		{
			msg.linear.x = x_vel; 
			msg.angular.z = 0;
			publisher_->publish(msg);		
		}
		// Keep turning if not reached last angular target		
        else if (std::abs(error) > th_tol)
        {
            msg.linear.x = 0.0;

            msg.angular.z = ramp_speed_from_error(
                error,
                /*v_max=*/th_vel,
                /*v_min=*/th_min_vel,
                /*slow_zone=*/th_slow_zone,
                /*tol=*/th_tol
            );

            publisher_->publish(msg);
        }
		// If done step, stop
		else
		{
			msg.linear.x = 0; //double(rand())/double(RAND_MAX); //fun
			msg.angular.z = 0; //2*double(rand())/double(RAND_MAX) - 1; //fun
			publisher_->publish(msg);
			last_state_complete = 1;
		}


		sequence_statemachine();		
		

		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	}
	
	void sequence_statemachine()
	{
		if (last_state_complete == 1)
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
		d_aim = distance;
		x_init = x_now;
		y_init = y_now;		
		count_++;		// advance state counter
		last_state_complete = 0;	
	}
	
	// Set the initial angle as where robot is heading and put new th_aim in place			
	void turn_angle(double angle)
	{
        th_init = th_now;
        th_target = wrap_angle(th_init + angle);
		count_++;		// advance state counter
		last_state_complete = 0;	
	}
	
	// Handle angle wrapping
    	double wrap_angle(double angle)
    	{
    	        angle = fmod(angle + M_PI,2*M_PI);
        	if (angle <= 0.0)
        	{
           		angle += 2*M_PI;
           	}           
        	return angle - M_PI;
    	}
    
	
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
    double th_tol = 0.03; // how close before the code delcares turning is "done"
	size_t count_ = 0;
    double th_slow_zone = 0.35;
    double th_min_vel   = 0.02;
	int last_state_complete = 1;
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

