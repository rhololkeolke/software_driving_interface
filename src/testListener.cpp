#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "software_driving_interface/HDI_feedback.h"
//#include "driving_msgs/HDI_feedback.h"

namespace sdi
{
   class SDI_Talker
   {
      bool messageProcessed;

      software_driving_interface::HDI_feedback feedbackMsg;

      void setWheelAngle(const std_msgs::Float64::ConstPtr& msg)
      {
         this->feedbackMsg.wheel_angle = msg.data;
         messageProcessed = true;
         ROS_INFO("Wheel Angle Set");
      }

      void setWheelForce(const std_msgs::Float64::ConstPtr& msg)
      {
         this->feedbackMsg.wheel_force = msg.data;
         messageProcessed = true;
         ROS_INFO("Wheel Force Set");
      }

      void setVibration(const std_msgs::Float64::ConstPtr& msg)
      {
         this->feedbackMsg.vibration = msg.data;
         messageProcessed = true;
         ROS_INFO("Vibration set");
      }

      public: int run(int argc, char **argv)
      {
         ros::init(argc, argv, "SDI_output");
         ros::NodeHandle handle;

         // Subscriber
//         ros::Subscriber subVibrationState = handle.subscribe("drc_vehicle_xp900/vibration/state", 1000, &SDI_Talker::setVibration, this);
//         ros::Subscriber subWheelForceState = handle.subscribe("drc_vehicle_xp900/hand_wheel_force/state", 1000, &SDI_Talker::setWheelForce, this);
         ros::Subscriber subHandWheelState = handle.subscriber("drc_vehicle_xp900/hand_wheel/state", 1000, &SDI_Talker::setWheelAngle, this);

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

   }; // end of SDI_Talker class
} // end of sdi namespace

int main(int argc, char **argv)
{
   sdi::SDI_Talker talker;
   talker.run(arc, argv);
}
