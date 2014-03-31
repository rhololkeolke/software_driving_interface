/*
 * SDI_cmd.h
 *
 *  Created on: Mar 21, 2014
 *      Author: democritus5589
 */

#ifndef _SDI_CMD_H_
#define _SDI_CMD_H_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "software_driving_interface/HDI_control.h"
#include <string>
using namespace std;
//#include "driving_msgs/HDI_control.h"


namespace sdi
{
   static const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;


   class SDI_Listener
   {
   public:
	   SDI_Listener()
	   {
		   messageProcessed = false;
	   }

	   int run(int argc, char **argv);

   private:
      bool messageProcessed;

      static const double MIN_GAS_PERCENT = 0.0;
      static const double MAX_GAS_PERCENT = 1.0;

      static const double MIN_BRAKE_PEDAL_PERCENT = 0.0;
      static const double MAX_BRAKE_PEDAL_PERCENT = 0.0;

      static const int VALID_DIRECTION_VALUES[4];// = { -1, 0, 1, 2 };

      static const double MIN_HAND_BRAKE_PERCENT = 0.0;
      static const double MAX_HAND_BRAKE_PERCENT = 1.0;

      static double MIN_WHEEL_POS() { return -5 * PI; };
      static double MAX_WHEEL_POS() { return 5 * PI; };

      static const int VALID_KEY_VALUES[2];// = { 0, 1 };

      static const double MIN_VIBRATION_PERCENT = 0.0;
      static const double MAX_VIBRATION_PERCENT = 1.0;

      std_msgs::Float64 gasPedalPercentMsg;
      std_msgs::Float64 brakePedalPercentMsg;
      std_msgs::Float64 wheelAngleMsg;
      std_msgs::Float64 handBrakePercentMsg;
      std_msgs::Int8 directionValueMsg;
      std_msgs::Int8 keyValueMsg;
      std_msgs::Float64 vibrationMsg;


      void extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg);

      void validateMsgInput();
      void logMessage(const software_driving_interface::HDI_control::ConstPtr& msg);
      void logOutboundMessage(string* topic, const std_msgs::Int8::ConstPtr& msg);
      void logOutboundMessage(string* topic, const std_msgs::Float64::ConstPtr& msg);
      void logMessage();

   }; // end of SDI_Listener class

   const int SDI_Listener::VALID_DIRECTION_VALUES[4] = { -1, 0, 1, 2 };
   const int SDI_Listener::VALID_KEY_VALUES[2] = { 0, 1 };

} // end of sdi namespace



#endif /* _TEST_TALKER_2_H_ */
