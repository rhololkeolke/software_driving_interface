#include "software_driving_interface/SDI_state.h"
#include "software_driving_interface/HDI_feedback.h"
//#include "driving_msgs/HDI_feedback.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include <math.h>

using namespace sdi;
using namespace std;

/***************************************************************

	Callbacks

***************************************************************/
void SDI_Talker::setWheelAngle(const std_msgs::Float64::ConstPtr& msg)
{
   logMessage("wheel_angle", msg);
   this->wheelAngle = msg->data;
//   this->feedbackMsg.wheel_angle = msg->data;
//   messageProcessed = true;
//   ROS_INFO("Wheel Angle Set");
}

void SDI_Talker::setWheelForce(const std_msgs::Float64::ConstPtr& msg)
{
   logMessage("wheel_force", msg);
   this->wheelForce = msg->data;
//   this->feedbackMsg.wheel_force = msg->data;
//   messageProcessed = true;
//   ROS_INFO("Wheel Force Set");
}

void SDI_Talker::setVibration(const std_msgs::Int8::ConstPtr& msg)
{
	logMessage("vibration", msg); // TODO: replace with more accurate msg
	logMessage("key", msg);

	//	engine is on, provide vibrations
	if (0 != msg->data)
	{
		this->vibration = 1.0;
//		this->feedbackMsg.vibration = 1.0;
	}
	//	engine is off, no vibrations
	else
	{
		this->vibration = 0.0;
//		this->feedbackMsg.vibration = 0.0;
	}

//   messageProcessed = true;
//   ROS_INFO("Vibration set");
}

/***************************************************************

	Logging

***************************************************************/
void SDI_Talker::logMessage(software_driving_interface::HDI_feedback& msg)//const software_driving_interface::HDI_feedback::ConstPtr& msg)
{
   stringstream ss;

   ss << "SDI messages sent to HDI and logged. Message Contents:\n";
   ss << "Wheel Position:\t" << msg.wheel_angle << "\n";
   ss << "Wheel Force:\t" << msg.wheel_force << "\n";
   ss << "Vibration:\t" << msg.vibration << "\n";

   ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());
}

void SDI_Talker::logMessage(string name, const std_msgs::Float64::ConstPtr& msg)
{
	stringstream ss;

	ss << "Sim message received by SDI and logged. Message contents:\n";

	if ("wheel_force" == name)
	{
		ss << "Wheel Force:\t";
	}
	else if ("wheel_angle" == name)
	{
		ss << "Wheel Angle:\t";
	}
	else
	{
		ss << name << "\t";
	}

	ss << msg->data << "\n";

	ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());
}

void SDI_Talker::logMessage(string name, const std_msgs::Int8::ConstPtr& msg)
{
	stringstream ss;

	ss << "Sim message received by SDI and logged. Message contents:\n";

	if ("vibration" == name)
	{
		ss << "Vibration Value:\t";
	}
	else if ("key" == name)
	{
		ss << "Key State:\t";
	}
	else
	{
		ss << name << "\t";
	}

	ss << msg->data << "\n";

	ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());
}

void SDI_Talker::logMessage(const nav_msgs::Odometry::ConstPtr& msg)
{
   stringstream ss;

   ss << "Sim message received by SDI and logged. Relevant message contents:\n";

   // Position
   ss << "Position (x, y, z):\t(" << msg->pose.pose.position.x;
   ss << ", " << msg->pose.pose.position.y;
   ss << ", " << msg->pose.pose.position.z;
   ss << ")\n";
  

   // Linear Velocity
   ss << "Linear velocity (x, y, z):\t(" << msg->twist.twist.linear.x;
   ss << ", " << msg->twist.twist.linear.y;
   ss << ", " << msg->twist.twist.linear.z;
   ss << ")\n";
  

   // Angular Velocity
   ss << "Angular velocity (x, y, z):\t(" << msg->twist.twist.angular.x;
   ss << ", " << msg->twist.twist.angular.y;
   ss << ", " << msg->twist.twist.angular.z;
   ss << ")\n";

   ROS_WARN_NAMED("Testing_WARN", ss.str().c_str());
}

/***************************************************************

	Running

***************************************************************/

void SDI_Talker::setVelocity(const nav_msgs::Odometry::ConstPtr& msg)
{
// geometry_msgs/PoseWithCovariance pose
//   geometry_msgs/Pose pose
//      geometry_msgs/Point position
//         float64/x
//         float64/y
//         float64/z
//      geomtery_msgs/Quaternion orientation
// geometry_msgs/TwistWithCovariance twist
//   geometry_msgs/Twist twist
//      geomtery_msgs/Vector3 linear
//         float64/x
//         float64/y
//         float64/z
//      geometry_msgs/Vector3 angular

   logMessage(msg);

   double temp1[] = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
   };
   this->atlasPosition = vector<double> (temp1, temp1 + sizeof(temp1) / sizeof(double));

   double temp2[] = {
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z
   };
   this->atlasLinearVelocity = vector<double> (temp2, temp2 + sizeof(temp2) / sizeof(double));

   double temp3[] = {
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z
   };
   this->atlasAngularVelocity = vector<double> (temp3, temp3 + sizeof(temp3) / sizeof(double));

}

void SDI_Talker::setHandWheelForce()
{
   this->atlasLinearVelocity;
   this->atlasAngularVelocity;
   this->wheelAngle;

// angle restricted to MIN_ANGLE and MAX_ANGLE
   double MIN_ANGLE = -7;
   double MAX_ANGLE = 7;

   double linearSpeed = 0;

   for (vector<double>::iterator iter = atlasLinearVelocity.begin(); atlasLinearVelocity.end() != iter; iter++)
   {
      linearSpeed += *iter * *iter;
   }

   linearSpeed = sqrt(linearSpeed);

   double angularSpeed = 0;

   for (vector<double>::iterator iter = atlasAngularVelocity.begin(); atlasAngularVelocity.end() != iter; iter++)
   {
      angularSpeed += *iter * *iter;
   }

   angularSpeed = sqrt(angularSpeed);

   this->wheelForce = linearSpeed * angularSpeed * wheelAngle;
//   this->feedbackMsg.wheel_force = wheelForce;
}

void SDI_Talker::fillFeedbackMsg()
{
   setHandWheelForce();
   this->feedbackMsg.wheel_force = wheelForce;
   this->feedbackMsg.wheel_angle = wheelAngle;
   this->feedbackMsg.vibration = vibration;
}

int SDI_Talker::run(int argc, char **argv)
{
	ros::init(argc, argv, "SDI_output");
	ros::NodeHandle handle;

	// Subscriber
	ros::Subscriber subHandWheelState = handle.subscribe("drc_vehicle_xp900/hand_wheel/state", 1000, &SDI_Talker::setWheelAngle, this);
	ros::Subscriber subKeyState = handle.subscribe("drc_vehicle_xp900/key/state", 1000, &SDI_Talker::setVibration, this);
	ros::Subscriber subAtlasHipState = handle.subscribe("ground_truth_odom", 1000, &SDI_Talker::setVelocity, this);

	// Publisher
	ros::Publisher pubHDIState = handle.advertise<software_driving_interface::HDI_feedback>("HDI/state", 1000);

	ros::Rate loop_rate(10);
	ros::spinOnce();
	
        int count = 0;
	while (ros::ok())
	{

           fillFeedbackMsg();
           logMessage(feedbackMsg);
           pubHDIState.publish(feedbackMsg);
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
