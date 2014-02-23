#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <pthread.h>

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
   ros::Rate loop_rate(1);
   
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
