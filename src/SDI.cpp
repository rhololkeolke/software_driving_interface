
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <pthread.h>
/*#include <queue>
#include <stdlib.h>

#include <software_driving_interface/HDI_control.h>
#include <software_driving_interface/HDI_feedback.h>
*/
/*
#include <math.h>
#include <stdlib.h>
#include <gazebo/common/common.hh>



namespace sdi
{

SoftwareDrivingInterface::SoftwareDrivingInterface()
{

}

SoftwareDrivingInterface::~SoftwareDrivingInterface()
{

}

void SoftwareDrivingInterface::Load()
{
   if (!ros::isInitialized())
   {
      ROS_ERROR("ROS is not initialized properly. Terminating now.");
      return;
   }

   this->rosNode = new ros::NodeHandle("");

   String modelName = "/drc_vehicle_xp900";

   // Subscribers
   ros::SubscribeOptions has_pedal_state_so = 
      modelName + "/gas_pedal/state", 100,
      boost::bind(static_cast< void (SoftwareDrivingInterface::*)
         (const std_msgs::Float64::ConstPtr&) > (
         &SoftwareDrivingInterface::
}

void SoftwareDrivingInterface::ConvertHDIMsgToSimMsgs()
{

}

common::Time SoftwareDrivingInterface::GetRosPublishPeriod()
{
   return this->rosPublishPeriod;
}

void SoftwareDrivingInterface::SetRosPublishRate(double hz)
{
   if (0.0 < hz)
   {
      this->rosPublishPeriod = 1.0 / hz;
   }
   else
   {
      this->rosPublishPeriod = 0.0;
   }
}

void SoftwareDrivingInterface::RosPublishStates()
{
   time_t currentTime = time(0);
   if ((currentTime - this->lastRosPublishTime) >= this->rosPublishPeriod)
   {
      this->lastRosPublishTime = currentTime; // need to add cast or change type

      std_msgs::Float64 msg_steer, msg_brake, msg_gas, msg_hand_brake;

      msg_steer.data = 0;
      msg_brake.data = 0;
      msg_gas.data = 0;
      msg_hand_brake.data = 0;
      
      this->pubGasPedalCmd(msg_gas);
      this->pubBrakePedalCmd(msg_brake);
      this->pubHandWheelCmd(msg_steer);
      this->pubHandBrakeCmd(msg_hand_brake);

      std_msgs::Int8 msg_key, msg_direction;

      msg_key.data = 0;
      msg_direction.data = 0;

      this->pubKeyCmd(msg_key);
      this->pubDirectionCmd(msg_direction);

      software_driving_interface::HDI_feedback msg_feedback;

      msg_feedback->wheel_angle = 0;
      msg_feedback->wheel_force = 0;
      msg_feedback->vibration = 0;

      this->pubHDIFeedback(msg_feedback);
      
   }
}

void SoftwareDrivingInterface::QueueThread()
{
   static const double timeout = 0.01;
   while (this->rosNode->ok())
   {
      this->queue.callAvailable(ros::WallDuration(timeout);
   }
}


}


*/




/////////////////////////////////////////////////////
/*


// Queues
std::queue<double> queueHandWheelCmd;
std::queue<double> queueGasPedalCmd;
std::queue<double> queueBrakePedalCmd;
std::queue<double> queueHandBrakeCmd;
std::queue<short> queueKeyCmd;
std::queue<short> queueDirectionCmd;

void HDICmdCallback(const software_driving_interface::HDI_control::ConstPtr& msg)
{
   queueHandWheelCmd.push(msg->wheel_angle);
   queueGasPedalCmd.push(msg->gas_pos);
   queueBrakePedalCmd.push(msg->brake_pos);
   queueDirectionCmd.push(msg->gear);
//   msg->vibration
}

void HDIStateCallback(const software_driving_interface::HDI_feedback::ConstPtr& msg)
{

}

double getHandWheelCmdValue()
{
   double value = 0;

   if (!queueHandWheelCmd.empty())
   {
      value = queueHandWheelCmd.front();
      if (1 < queueHandWheelCmd.size())
      {
         queueHandWheelCmd.pop();
      }
   }

   return value;
}

double getGasPedalCmdValue()
{
   double value = 0;

   if (!queueGasPedalCmd.empty())
   {
      value = queueGasPedalCmd.front();
      if (1 < queueGasPedalCmd.size())
      {
         queueGasPedalCmd.pop();
      }
   }

   return value;

}

double getBrakePedalCmdValue()
{
   double value = 0;

   if (!queueBrakePedalCmd.empty())
   {
      value = queueBrakePedalCmd.front();
      if (1 < queueBrakePedalCmd.size())
      {
         queueBrakePedalCmd.pop();
      }
   }

   return value;

}

double getHandBrakeCmdValue()
{
   double value = 0;

   if (!queueHandBrakeCmd.empty())
   {
      value = queueHandBrakeCmd.front();
      if (1 < queueHandBrakeCmd.size())
      {
         queueHandBrakeCmd.pop();
      }
   }

   return value;

}

short getKeyCmdValue()
{
   short value = 0;

   if (!queueKeyCmd.empty())
   {
      value = queueKeyCmd.front();
      if (1 < queueKeyCmd.size())
      {
         queueKeyCmd.pop();
      }
   }

   return value;

}

short getDirectionCmdValue()
{
   short value = 0;

   if (!queueDirectionCmd.empty())
   {
      value = queueDirectionCmd.front();
      if (1 < queueDirectionCmd.size())
      {
         queueDirectionCmd.pop();
      }
   }

   return value;

}

void cmdLoop(int argc, char **argv)
{
   ros::init(argc, argv, "SDI_cmd");
   ros::NodeHandle handle;

   ros::Subscriber subHDICmd = handle.subscribe("HDI_cmd", 1000, HDICmdCallback);


   ros::Publisher pubHandWheelCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_wheel/cmd", 1000);
   ros::Publisher pubGasPedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/gas_pedal/cmd", 1000);
   ros::Publisher pubBrakePedalCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/brake_pedal/cmd", 1000);
   ros::Publisher pubHandBrakeCmd = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_brake/cmd", 1000);
   ros::Publisher pubKeyCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/key/cmd", 1000);
   ros::Publisher pubDirectionCmd = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/direction/cmd", 1000);

   ros::Rate loop_rate(10);
   
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      std_msgs::Float64 msg_steer, msg_brake, msg_gas, msg_hand_brake;

      msg_steer.data = getHandWheelCmdValue();
      msg_brake.data = getBrakePedalCmdValue();
      msg_gas.data = getGasPedalCmdValue();
      msg_hand_brake.data = getHandBrakeCmdValue();
      
      pubGasPedalCmd.publish(msg_gas);
      pubBrakePedalCmd.publish(msg_brake);
      pubHandWheelCmd.publish(msg_steer);
      pubHandBrakeCmd.publish(msg_hand_brake);

      std_msgs::Int8 msg_key, msg_direction;

      msg_key.data = getKeyCmdValue();
      msg_direction.data = getDirectionCmdValue();

      pubKeyCmd.publish(msg_key);
      pubDirectionCmd.publish(msg_direction);

//      software_driving_interface::HDI_feedback msg_feedback;

//      msg_feedback->wheel_angle = 0;
//      msg_feedback->wheel_force = 0;
//      msg_feedback->vibration = 0;

//      this->pubHDIFeedback(msg_feedback);


      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }

}







*/








void testTalker(int argc, char **argv);
void *keyTalker(void *ptr);
void *wheelTalker(void *ptr);
void *gasTalker(void *ptr);
void *handBrakeTalker(void *ptr);
void testThreads(void *arg);
void *print_message_function(void *ptr);
void *testTalker2(void *ptr);



struct inputContainer
{
   int argc;
   char **argv;
};

int main(int argc, char **argv)
{
   inputContainer inputWrapper;
   inputWrapper.argc = argc;
   inputWrapper.argv = argv;
//   testTalker(argc, argv);
//   const char *message = "Test message";
   testThreads((void *) &inputWrapper);
//   testTalker2((void *) &inputWrapper);
//   keyTalker(argc, argv);
//   wheelTalker(argc, argv);
//   gasTalker(argc, argv);
//   handBrakeTalker(argc, argv);
   return 0;
}

///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void testThreads(void *arg)
{
   pthread_t gasThread, handBrakeThread, wheelThread;
   const char *gasMessage = "Gas Position: ";
   const char *handBrakeMessage = "Hand Brake Position: ";
   const char *wheelMessage = "Wheel Position: ";

   int gasThreadError, handBrakeThreadError, wheelThreadError;

   gasThreadError = pthread_create(&gasThread, NULL, gasTalker, arg);
   handBrakeThreadError = pthread_create(&handBrakeThread, NULL, handBrakeTalker, arg);
   wheelThreadError = pthread_create(&wheelThread, NULL, wheelTalker, arg);

   pthread_join(gasThread, NULL);
   pthread_join(handBrakeThread, NULL);
   pthread_join(wheelThread, NULL);

   printf("Gas Thread returned: %d\n", gasThreadError);
   printf("Hand Brake Thread returned: %d\n", handBrakeThreadError);
   printf("Wheel Thread returned: %d\n", wheelThreadError);
}

void *print_message_function(void *ptr)
{
   char *message;
   message = (char *) ptr;
   printf("%s \n", message);
   return message;
}

void testTalker(int argc, char **argv)
{
   printf("Starting talker with %d\n", argc);
   ros::init(argc, argv, "gasTalker", ros::init_options::AnonymousName);
   ros::NodeHandle handle;
printf("Created handle\n");
   ros::Publisher publisher = handle.advertise<std_msgs::String>("chatter", 1000);
   ros::Publisher publisher2 = handle.advertise<std_msgs::String>("chatter2", 1000);
   ros::Rate loop_rate(10);
printf("Starting loop now\n");
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      printf("Count: %d\n", count);
      std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      publisher.publish(msg);
      publisher2.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
      printf("Count: %d\n", count);
   }
   printf("Loop ended with count: %d.\n", count);
//   return 0;
}

void *testTalker2(void *ptr)
{
   inputContainer* container = (inputContainer *)ptr;
   printf("Starting second talker now\n");
   testTalker(container->argc, container->argv);
   return ptr;
}

// hand wheel
void *wheelTalker(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;

   ros::init(container->argc, container->argv, "wheelTalker");
   ros::NodeHandle handle;

   ros::Publisher publisher = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_wheel/cmd", 1000);
   ros::Rate loop_rate(10);
   
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      std_msgs::Float64 msg;
      msg.data = -3.14 + (static_cast <double> (rand())) / (static_cast <double> (RAND_MAX/(3.14-(-3.14))));
      ROS_INFO("Count: %d\tWheel value: %f", count, msg.data);
      publisher.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }
   return ptr;
}

// gas pedal
void *gasTalker(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;

   ros::init(container->argc, container->argv, "gasTalker");
   ros::NodeHandle handle;

   ros::Publisher publisher = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/gas_pedal/cmd", 1000);
   ros::Rate loop_rate(10);
   
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      std_msgs::Float64 msg;
      msg.data = (static_cast <double> (rand())) / (static_cast <double> (RAND_MAX));
      ROS_INFO("Count: %d\tGas value: %f", count, msg.data);
      publisher.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }
   return ptr;
}

// hand brake
void *handBrakeTalker(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;

   ros::init(container->argc, container->argv, "handBrakeTalker");
   ros::NodeHandle handle;

   ros::Publisher publisher = handle.advertise<std_msgs::Float64>("drc_vehicle_xp900/hand_brake/cmd", 1000);
   ros::Rate loop_rate(10);
   
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      std_msgs::Float64 msg;
      msg.data = 0; //(static_cast <double> (rand())) / (static_cast <double> (RAND_MAX));
      ROS_INFO("Count: %d\tHand Brake value: %f", count, msg.data);
      publisher.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }
   return ptr;
}

// key
void *keyTalker(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;

   ros::init(container->argc, container->argv, "keyTalker");
   ros::NodeHandle handle;

   ros::Publisher publisher = handle.advertise<std_msgs::Int8>("drc_vehicle_xp900/key/cmd", 1000);
   ros::Rate loop_rate(10);
   
   int count = 0;
   while (ros::ok() && (10 > count))
   {
      std_msgs::Int8 msg;
      msg.data = 1;
      ROS_INFO("%d", msg.data);
      publisher.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
   }
   return ptr;
}

