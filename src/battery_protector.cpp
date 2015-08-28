/*! \file battery_protector.cpp
* \brief Battery protector script
*
* This package is NOT used in the current version of the package. However, it is a nice idea to integrate such functionality. This package gives an audible warning if the battery gets low.
*
* TODO:
* - Test and update
*/

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>				// ROS header
#include <sound_play/sound_play.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <unistd.h>

/*! Battery protector class, giving an audible warning if battery gets low */
class BatteryProtector
{
private:
	ros::NodeHandle node_handle_; 			/**< Node handle */
	sound_play::SoundClient sc;				/**< Sound client */
	ros::Subscriber battery_subscriber_;	/**< Subscriber for battery percentages */
	float treshold_;						/**< Threshold used for warnings */
public:
	//! Init. Say that you are powered on.
	BatteryProtector()
	{
		ros::NodeHandle params("~");
		treshold_ = 20;			// Default value for threshold: 20 percent.
		battery_subscriber_ = node_handle_.subscribe<std_msgs::Float32>("ardrone/navdata/batteryPercent",1, &BatteryProtector::batteryCallback, this);
		usleep(3*1000000);
		sc.say("Battery protector initialized. Fly safe!");
	}

	~BatteryProtector()
	{
	}

	//! Check if battery is low. If so: say land.
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