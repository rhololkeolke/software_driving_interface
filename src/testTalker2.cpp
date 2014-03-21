#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "software_driving_interface/HDI_control.h"

//#include "driving_msgs/HDI_control.h"


namespace sdi
{
   static const double PI = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;

   class SDI_Listener
   {
      bool messageProcessed;



      static const double MIN_GAS_PERCENT = 0.0;
      static const double MAX_GAS_PERCENT = 1.0;

      static const double MIN_BRAKE_PEDAL_PERCENT = 0.0;
      static const double MAX_BRAKE_PEDAL_PERCENT = 0.0;

      static const int VALID_DIRECTION_VALUES[];// = { -1, 0, 1, 2 };

      static const double MIN_HAND_BRAKE_PERCENT = 0.0;
      static const double MAX_HAND_BRAKE_PERCENT = 1.0;

      static const double MIN_WHEEL_POS = -5 * PI;
      static const double MAX_WHEEL_POS = 5 * PI;

      static const int* VALID_KEY_VALUES;// = { 0, 1 };

      static const double MIN_VIBRATION_PERCENT = 0.0;
      static const double MAX_VIBRATION_PERCENT = 1.0;

      std_msgs::Float64 gasPedalPercentMsg;
      std_msgs::Float64 brakePedalPercentMsg;
      std_msgs::Float64 wheelAngleMsg;
      std_msgs::Float64 handBrakePercentMsg;
      std_msgs::Int8 directionValueMsg;
      std_msgs::Int8 keyValueMsg;
      std_msgs::Float64 vibrationMsg;


      void extractMsgValues(const software_driving_interface::HDI_control::ConstPtr& msg)
      {
         this->gasPedalPercentMsg.data = msg->gas_pos;
         this->brakePedalPercentMsg.data = msg->brake_pos;
         this->wheelAngleMsg.data = msg->wheel_angle;
         this->handBrakePercentMsg.data = 0;
         this->directionValueMsg.data = msg->gear;
         this->keyValueMsg.data = 1;
         this->vibrationMsg.data = msg->vibration;

         messageProcessed = true;

         ROS_INFO("ExtractMsgValues method completed.");
         ROS_INFO("Gas: %f\tBrake: %f\tWheel: %f\tHand: %f\tDir: %d\tKey: %d\tVib: %f",
        	gasPedalPercentMsg.data,
        	brakePedalPercentMsg.data,
        	wheelAngleMsg.data,
        	handBrakePercentMsg.data,
        	directionValueMsg.data,
        	keyValueMsg.data,
        	vibrationMsg.data
         	 );
      }

      void validateMsgInput()
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

		  if (MIN_WHEEL_POS > this->wheelAngleMsg.data)
		  {
			  ROS_WARN("Value of wheel angle received from HDI (%f) is LESS than the minimum acceptable value (%f). Value set to min.", wheelAngleMsg.data, MIN_WHEEL_POS);
			  wheelAngleMsg.data = MIN_WHEEL_POS;
		  }
		  else if (MAX_WHEEL_POS < this->wheelAngleMsg.data)
		  {
			  ROS_WARN("Value of wheel angle received from HDI (%f) is GREATER than the maximum acceptable value (%f). Value set to max.", wheelAngleMsg.data, MAX_WHEEL_POS);
			  wheelAngleMsg.data = MAX_WHEEL_POS;
		  }


		  int* searchResult = std::find(VALID_DIRECTION_VALUES, VALID_DIRECTION_VALUES + 4, directionValueMsg.data);

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

      public: int run(int argc, char **argv)
      {
    	  vehicleOff = false;
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

         int count = 0;
         while (ros::ok())
         {
            if (messageProcessed)
            {

               messageProcessed = false;

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
