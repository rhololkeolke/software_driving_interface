/*
 * testListener.h
 *
 *  Created on: Mar 21, 2014
 *      Author: democritus5589
 */

#ifndef _TEST_LISTENER_H_
#define _TEST_LISTENER_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "software_driving_interface/HDI_feedback.h"
//#include "driving_msgs/HDI_feedback.h"
#include <string>

using namespace std;

namespace sdi
{
   class SDI_Talker
   {
   public:
	   int run(int argc, char **argv);

   private:
      bool messageProcessed;

      software_driving_interface::HDI_feedback feedbackMsg;

      void setWheelAngle(const std_msgs::Float64::ConstPtr& msg);

      void setWheelForce(const std_msgs::Float64::ConstPtr& msg);

      void setVibration(const std_msgs::Int8::ConstPtr& msg);

      void logMessage(string name, const std_msgs::Int8::ConstPtr& msg);
      void logMessage(string name, const std_msgs::Float64::ConstPtr& msg);
      void logMessage(software_driving_interface::HDI_feedback& msg);//const software_driving_interface::HDI_feedback::ConstPtr& msg);

   }; // end of SDI_Talker class
} // end of sdi namespace



#endif /* _TEST_LISTENER_H_ */
