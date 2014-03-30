#include "software_driving_interface/testListener.h"
#include "software_driving_interface/HDI_feedback.h"
//#include "driving_msgs/HDI_feedback.h"

using namespace sdi;
using namespace std;

void SDI_Talker::setWheelAngle(const std_msgs::Float64::ConstPtr& msg)
{
   this->feedbackMsg.wheel_angle = msg->data;
   messageProcessed = true;
   ROS_INFO("Wheel Angle Set");
}

void SDI_Talker::setWheelForce(const std_msgs::Float64::ConstPtr& msg)
{
   this->feedbackMsg.wheel_force = msg->data;
   messageProcessed = true;
   ROS_INFO("Wheel Force Set");
}

void SDI_Talker::setVibration(const std_msgs::Int8::ConstPtr& msg)
{
	//	engine is on, provide vibrations
	if (0 != msg->data)
	{
		this->feedbackMsg.vibration = 1.0;
	}
	//	engine is off, no vibrations
	else
	{
		this->feedbackMsg.vibration = 0.0;
	}

   messageProcessed = true;
   ROS_INFO("Vibration set");
}

int SDI_Talker::run(int argc, char **argv)
{
	ros::init(argc, argv, "SDI_output");
	ros::NodeHandle handle;

	// Subscriber
	//         ros::Subscriber subVibrationState = handle.subscribe("drc_vehicle_xp900/vibration/state", 1000, &SDI_Talker::setVibration, this);
	//         ros::Subscriber subWheelForceState = handle.subscribe("drc_vehicle_xp900/hand_wheel_force/state", 1000, &SDI_Talker::setWheelForce, this);
	ros::Subscriber subHandWheelState = handle.subscribe("drc_vehicle_xp900/hand_wheel/state", 1000, &SDI_Talker::setWheelAngle, this);
	ros::Subscriber subKeyState = handle.subscribe("drc_vehicle_xp900/key/state", 1000, &SDI_Talker::setVibration, this);
//	ros::Subscriber subVelocityState = handle.subscribe("drc_vehicle_xp900/velocity/state", 1000, &SDI_Talker::setVelocity, this);

	// Publisher
	ros::Publisher pubHDIState = handle.advertise<software_driving_interface::HDI_feedback>("HDI/state", 1000);

	ros::Rate loop_rate(10);
	ros::spinOnce();

	int count = 0;
	while (ros::ok())
	{
		if (messageProcessed)
		{
			messageProcessed = false;
			pubHDIState.publish(feedbackMsg);
		}

		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	return 0;
}

int main(int argc, char **argv)
{
   sdi::SDI_Talker talker;
   talker.run(argc, argv);
}
