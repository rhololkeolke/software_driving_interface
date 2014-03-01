#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "software_driving_interface/HDI_control.h"
//#include "driving_msgs/HDI_control.h"

namespace sdi
{
   class SDI_Listener
   {
      bool messageProcessed;

      std_msgs::Float64 gasPedalPercentMsg;
      std_msgs::Float64 brakePedalPercentMsg;
      std_msgs::Float64 wheelAngleMsg;
      std_msgs::Float64 handBrakePercentMsg;
      std_msgs::Int8 directionValueMsg;
      std_msgs::Int8 keyValueMsg;


      void extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg)
      {
         this->gasPedalPercentMsg.data = msg->gas_pos;
         this->brakePedalPercentMsg.data = msg->brake_pos;
         this->wheelAngleMsg.data = msg->wheel_angle;
         this->handBrakePercentMsg.data = 0;
         this->directionValueMsg.data = msg->gear;
         this->keyValueMsg.data = 1;

         messageProcessed = true;

         ROS_INFO("ExtractMsgValues method completed.");
      }

      public: int run(int argc, char **argv)
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


         ros::Rate loop_rate(10);
         ros::spinOnce();

         int count = 0;
         while (ros::ok())
         {
            if (messageProcessed)
            {
               messageProcessed = false;
//               pubKeyCmd.publish(keyValueMsg);
               pubHandBrakeCmd.publish(handBrakePercentMsg);
               pubGasPedalCmd.publish(gasPedalPercentMsg);
               pubHandWheelCmd.publish(wheelAngleMsg); 
            }

            ros::spinOnce();
            loop_rate.sleep();
            count++;
         }

         return 0;
      }

   }; // end of SDI_Listener class

} // end of sdi namespace

int main(int argc, char **argv)
{
   sdi::SDI_Listener listener;
   listener.run(argc, argv);
}
