#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <termios.h> // This and down is for the keyboard input
#include <stdio.h>
#include <sys/poll.h>


// 2D!
#define KEY_FORWARD 0x77 // W
#define KEY_LEFT 0x61 // A
#define KEY_BACKWARD 0x73 // S
#define KEY_RIGHT 0x64 // D
#define KEY_UP 0x6B // K
#define KEY_DOWN 0x6D // M
#define KEY_RESET 0x72 // R
#define KEY_LAUNCH 0x6F // O
#define KEY_LAND 0x6C // L

geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
int speed_x = 0;
int speed_y = 0;
int speed_z = 0;
int turn = 0;
double speed_factor = 0.5;
double turn_factor = 0.5;
bool dirty = false; // To make sure to stop the robot after a command is given
int kfd = 0;
struct termios cooked, raw;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fly_from_keyboard");
	ros::NodeHandle node;
	ros::Rate loop_rate(50); // Check if fast enough / consistent with launch files
	ros::Publisher pub_twist; // We need to publish some twists
	ros::Publisher pub_takeoff; // and some empty messages
	ros::Publisher pub_land; // and some empty messages
	ros::Publisher pub_reset; // and some empty messages

	pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel",1); // command velocity
	pub_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff",1); // command velocity
	pub_land = node.advertise<std_msgs::Empty>("ardrone/land",1); // command velocity
	pub_reset = node.advertise<std_msgs::Empty>("ardrone/reset",1); // command velocity
	

	ROS_INFO("Waiting for keyboard input");
	
	char c;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard.");
	
	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;

	for(;;)
	{
		int num;
		if ((num = poll(&ufd, 1, 250)) < 0)
		{
			perror("poll():");
			return 0;
		}
		else if(num > 0)
		{
			if(read(kfd, &c, 1) < 0)
			{
				perror("read():");
				return 0;
			}
		}
		else
		{
			if(dirty == true)
			{
				// Stop robot and flush vars
				speed_x = 0;
				speed_y = 0;
				speed_z = 0;
				turn = 0;		
				twist_msg.linear.x= 0.0;
				twist_msg.linear.y= 0.0;
				twist_msg.linear.z= 0.0;
				twist_msg.angular.z = 0.0;
				pub_twist.publish(twist_msg);
				dirty = false;
			}

			continue;
		}
		ROS_INFO("%c",c);
		switch(c)
		{

			case KEY_LAUNCH:
				pub_takeoff.publish(emp_msg);
				dirty = true;
				break;
			case KEY_RESET:
				pub_reset.publish(emp_msg);
				dirty = true;
				break;
			case KEY_LAND:
				pub_land.publish(emp_msg);
				dirty = true;
				break;
			case KEY_FORWARD:
				speed_x = 1;
				dirty = true;
				break;
			case KEY_BACKWARD:
				speed_x = -1;
				dirty = true;
				break;
			case KEY_UP:
				speed_z = 1;
				dirty = true;
				break;
			case KEY_DOWN:
				speed_z = -1;
				dirty = true;
				break;
			case KEY_LEFT:
				speed_y = 1;
				dirty = true;
				break;
			case KEY_RIGHT:
				speed_y = -1;
				dirty = true;
				break;
			default:
				speed_x = 0;
				speed_y = 0;
				speed_z = 0;
				turn = 0;
			
		}

		twist_msg.linear.x = speed_x*speed_factor;
		twist_msg.linear.y = speed_y*speed_factor;
		twist_msg.linear.z = speed_z*speed_factor;
		twist_msg.angular.z = turn*turn_factor;
		pub_twist.publish(twist_msg);
		ros::spinOnce(); // ??
		loop_rate.sleep();
	}
ROS_ERROR("ROS:ok() failed - Node Closing");
tcsetattr(kfd, TCSAFLUSH, &cooked);
return(0);
} // main
