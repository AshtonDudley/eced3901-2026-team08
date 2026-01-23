/*
Code for DT1 - Edited to navigate maze
V. Sieben && Megan Neville
Version 1.1
Date: Jan 23, 2026
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>

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
		
		
		// Keep moving if not reached last distance target
		if (d_now < d_aim)
		{
			msg.linear.x = x_vel; 
			msg.angular.z = 0;
			publisher_->publish(msg);		
		}
		// Keep turning if not reached last angular target		
		else if ( abs(wrap_angle(th_now - th_init)) < th_aim)
		{
			msg.linear.x = 0; 
			msg.angular.z = th_vel;
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
			    move_distance(1.1); //move the robot forwards
			    break;
			  case 1:
			    turn_angle(M_PI/2.05);  //turn to avoid wall
			    break;
			  case 2:
			    move_distance(0.40);  //second movement for the robot
			    break;
			  case 3:
			    turn_angle(M_PI/2.05); //turn the robot again
				break;
			  case 4:
				turn_angle(M_PI/2.05);
				break;
			  case 5:
				turn_angle(M_PI/2.05);
			    break;
			  case 6:
			    move_distance(1.0);  //drive toward obstacle
			    break;			    
			  case 7:
			    turn_angle(M_PI/2.05); //turn the robot again
				break;
			  case 8:
				turn_angle(M_PI/2.05);
				break;
			  case 9:
				turn_angle(M_PI/2.05);
			    break;
			  case 10:
			    move_distance(0.40); //drive past wall
			    break;			    
			  case 11:
			    turn_angle(M_PI/2.05);  //turn towards cargo
			    break;  
              case 12:
                move_distance(1.2); //at cargo?
			  case 13: //start truning around
			    turn_angle(M_PI/2.05); //turn the robot again
				break;			  
			  case 14:
			    turn_angle(M_PI/2.05); //robot did 180
				break;
			  case 15:
			    move_distance(1.2);
				break;
			  case 16:
				turn_angle(M_PI/2.05);
				break;
			  case 17:
				turn_angle(M_PI/2.05);
				break;
			  case 18:
				turn_angle(M_PI/2.05); //should be facing correct direction
				break;
			  case 19:
			    move_distance(0.4);
				break;
			  case 20:
			    turn_angle(M_PI/2.05);
				break;
			  case 21:
			    move_distance(1.0);
				break:
			  case 22:
			    turn_angle(M_PI/2.05);
				break;
			  case 23:
			    move_distance(0.4);
			    break:
			  case 24:
			    turn_angle(M_PI/2.05);
			    break;
			  case 24:
			    turn_angle(M_PI/2.05);
			    break:
			  case 26:
			    turn_angle(M_PI/2.05);
			    break;
			  case 27:
			    move_distance(1.1); //should be back at start?
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
		th_aim = angle;
		th_init = th_now;
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
	double x_vel = 0.2, th_vel = 0.1; //turn velocity
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0, th_now = 0, th_init = 0;
	double d_now = 0, d_aim = 0, th_aim = 0;
	double q_x = 0, q_y = 0, q_z = 0, q_w = 0; 
	size_t count_ = 0;
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

