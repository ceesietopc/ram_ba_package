#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <unistd.h>

class BatteryProtector
{
private:
	ros::NodeHandle node_handle_;
	sound_play::SoundClient sc;
	ros::Subscriber battery_subscriber_;
	float treshold_;
public:
	BatteryProtector()
	{
		ros::NodeHandle params("~");
		treshold_ = 20;
		battery_subscriber_ = node_handle_.subscribe<std_msgs::Float32>("ardrone/navdata/batteryPercent",1, &BatteryProtector::batteryCallback, this);
		usleep(3*1000000);
		sc.say("Battery protector initialized. Fly safe!");
	}

	~BatteryProtector()
	{
	}

	void batteryCallback(const std_msgs::Float32 percent)
	{
		ROS_INFO("Battery charge remaining: %f",percent.data);
		if(percent.data < treshold_)
		{
			sc.say("Battery low. Please land.");
			usleep(5*1000000);
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "battery_protector");
	BatteryProtector battery_protector;
	ros::spin();

	return 0;
}