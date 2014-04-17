#include "software_driving_interface/SDI_cmd.h"
#include <sstream>
#include <string>

//#include "driving_msgs/HDI_control.h"

using namespace sdi;
using namespace std;

/*******************************************************

	Callback

*******************************************************/
void SDI_Listener::extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg)
{
	SDI_Listener::logMessage(msg);

	this->gasPedalPercentMsg.data = msg->gas_pos;
	this->brakePedalPercentMsg.data = msg->brake_pos;
	this->wheelAngleMsg.data = msg->wheel_angle;
	this->handBrakePercentMsg.data = 0;
	this->directionValueMsg.data = msg->gear;
	this->keyValueMsg.data = 1;
	this->vibrationMsg.data = msg->vibration;

	messageProcessed = true;

/*	ROS_INFO("ExtractMsgValues method completed.");
	ROS_INFO("Gas: %f\tBrake: %f\tWheel: %f\tHand: %f\tDir: %d\tKey: %d\tVib: %f",
			gasPedalPercentMsg.data,
			brakePedalPercentMsg.data,
			wheelAngleMsg.data,
			handBrakePercentMsg.data,
			directionValueMsg.data,
			keyValueMsg.data,
			vibrationMsg.data
		);
*/
}

/*******************************************************

	Logging

*******************************************************/
void SDI_Listener::logMessage(const software_driving_interface::HDI_control::ConstPtr& msg)
{
	stringstream ss;
	ss << "HDI message received by SDI and logged. Message contents:\n";
	ss << "wheel_angle:\t" << msg->wheel_angle << "\n";
	ss << "gas_pos:\t" << msg->gas_pos << "\n";
	ss << "brake_pos:\t" << msg->brake_pos << "\n";
	ss << "gear:\t" << msg->gear << "\n";
	ss << "vibration:\t" << msg->vibration << "\n";

//	ROS_INFO("%s", ss.str().c_str());
	ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());
}

void SDI_Listener::logMessage()//SDI_Listener& controller)
{

	SDI_Listener& controller = *this;
	
	stringstream ss;
	ss << "SDI messages sent to simulator and logged. Message contents:\n";
	ss << "Gas Pedal %:\t" << controller.gasPedalPercentMsg.data << "\n";
	ss << "Brake Pedal %:\t" << controller.brakePedalPercentMsg.data << "\n";
	ss << "Steering Wheel Angle (rad):\t" << controller.wheelAngleMsg.data << "\n";
	ss << "Hand Brake %:\t" << controller.handBrakePercentMsg.data << "\n";
	ss << "Direction Value:\t" << controller.directionValueMsg.data << "\n";
	ss << "Key Value:\t" << controller.keyValueMsg.data;

	if (!((0 == keyValueMsg.data) || ((1 == keyValueMsg.data) && (0 == directionValueMsg.data))))
	{
		ss << "\t(Not sent to sim)";
	}

	ss << "\n";

//	ss << "Vibration Value:\t" << controller->vibrationMsg.data << "\n";

//	ROS_INFO("%s", ss.str().c_str());
	ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());

}


/*******************************************************

	Running

*******************************************************/

void SDI_Listener::validateMsgInput()
{
	if (MIN_GAS_PERCENT > this->gasPedalPercentMsg.data)
	{
		ROS_WARN("Value of gas position received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", gasPedalPercentMsg.data, MIN_GAS_PERCENT);
		gasPedalPercentMsg.data = MIN_GAS_PERCENT;
	}
	else if (MAX_GAS_PERCENT < this->gasPedalPercentMsg.data)
	{
		ROS_WARN("Value of gas position received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", gasPedalPercentMsg.data, MAX_GAS_PERCENT);
		gasPedalPercentMsg.data = MAX_GAS_PERCENT;
	}

	if (MIN_BRAKE_PEDAL_PERCENT > this->brakePedalPercentMsg.data)
	{
		ROS_WARN("Value of brake position received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", brakePedalPercentMsg.data, MIN_BRAKE_PEDAL_PERCENT);
		brakePedalPercentMsg.data = MIN_BRAKE_PEDAL_PERCENT;
	}
	else if (MAX_BRAKE_PEDAL_PERCENT < this->brakePedalPercentMsg.data)
	{
		ROS_WARN("Value of brake position received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", brakePedalPercentMsg.data, MAX_BRAKE_PEDAL_PERCENT);
		brakePedalPercentMsg.data = MAX_BRAKE_PEDAL_PERCENT;
	}

	if (MIN_WHEEL_POS() > this->wheelAngleMsg.data)
	{
		ROS_WARN("Value of wheel angle received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", wheelAngleMsg.data, MIN_WHEEL_POS());
		wheelAngleMsg.data = MIN_WHEEL_POS();
	}
	else if (MAX_WHEEL_POS() < this->wheelAngleMsg.data)
	{
		ROS_WARN("Value of wheel angle received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", wheelAngleMsg.data, MAX_WHEEL_POS());
		wheelAngleMsg.data = MAX_WHEEL_POS();
	}


	const int* searchResult = std::find(VALID_DIRECTION_VALUES, VALID_DIRECTION_VALUES + 4, directionValueMsg.data);

	if ((!searchResult) && (searchResult >= VALID_DIRECTION_VALUES + 4))
	{
		ROS_WARN("Value of vehicle direction received from HDI (%f) is not VALID. Acceptable values are (-1: REVERSE, 0: NEUTRAL, 1: FORWARD, 2: PARK). Value set to max.", wheelAngleMsg.data, MAX_WHEEL_POS);
		directionValueMsg.data = 0;
	}

	if (2 == directionValueMsg.data)
	{
		handBrakePercentMsg.data = 1.0;
		directionValueMsg.data = 0.0;
	}
	else
	{
		handBrakePercentMsg.data = 0.0;
	}

	if (MIN_VIBRATION_PERCENT > this->vibrationMsg.data)
	{
		ROS_WARN("Value of vibration received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", vibrationMsg.data, MIN_VIBRATION_PERCENT);
		vibrationMsg.data = MIN_VIBRATION_PERCENT;
	}
	else if (MAX_VIBRATION_PERCENT < this->vibrationMsg.data)
	{
		ROS_WARN("Value of vibration received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", vibrationMsg.data, MAX_VIBRATION_PERCENT);
		vibrationMsg.data = MAX_VIBRATION_PERCENT;
	}
}

int SDI_Listener::run(int argc, char **argv)
{
	ros::init(argc, argv, "SDI_input");
	ros::NodeHandle handle;

	// Subscriber
	ros::Subscriber subHDICmd = handle.subscribe("HDI/cmd", 1000, &SDI_Listener::extractMsgValues, this);

	// Publisher
	ros::Publisher pubKeyCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/key/cmd", 1000);
	ros::Publisher pubHandBrakeCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_brake/cmd", 1000);
	ros::Publisher pubGasPedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/gas_pedal/cmd", 1000);
	ros::Publisher pubHandWheelCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_wheel/cmd", 1000);
	ros::Publisher pubBrakePedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/brake_pedal/cmd", 1000);
	ros::Publisher pubVehicleDirectionCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/direction/cmd", 1000);

	ros::Rate loop_rate(10);
	ros::spinOnce();

	while (ros::ok())
	{
		if (messageProcessed)
		{
			messageProcessed = false;
			logMessage();

			//	if key = on and vehicle is in neutral, publish
			//	if key is off, publish
			if ((0 == keyValueMsg.data) || ((1 == keyValueMsg.data) && (0 == directionValueMsg.data)))
			{
				pubKeyCmd.publish(keyValueMsg);
			}

			pubHandBrakeCmd.publish(handBrakePercentMsg);
			pubGasPedalCmd.publish(gasPedalPercentMsg);
			pubHandWheelCmd.publish(wheelAngleMsg);

			pubBrakePedalCmd.publish(brakePedalPercentMsg);
			pubVehicleDirectionCmd.publish(directionValueMsg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


int main(int argc, char **argv)
{
   sdi::SDI_Listener listener;
   listener.run(argc, argv);
}
