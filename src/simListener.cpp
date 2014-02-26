#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"

void testListener(int argc, char **argv);
void *keyListener(void *ptr);
void *wheelListener(void *ptr);
void *gasListener(void *ptr);
void *handBrakeListener(void *ptr);
void chatterCallback(const std_msgs::Float64::ConstPtr& msg);
void chatterCallback2(const std_msgs::Int8::ConstPtr& msgs);
void testThreads(void *arg);

struct inputContainer
{
   int argc;
   char **argv;
};

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
   ROS_INFO("I heard: [%f]:", msg->data);
}

int main(int argc, char **argv)
{
   inputContainer inputWrapper;
   inputWrapper.argc = argc;
   inputWrapper.argv = argv;

//   testListener(argc, argv);
   testThreads((void *) &inputWrapper);

   return 0;
}

void testListener(int argc, char **argv)
{
   ros::init(argc, argv, "gasListener");
   ros::NodeHandle handle;
   ros::Subscriber subscriber = handle.subscribe("drc_vehicle_xp900/gas_pedal/state", 1000, chatterCallback);
   ros::spin();
}

void testThreads(void *arg)
{
   pthread_t gasThread, handBrakeThread, wheelThread;
   int gasThreadError, handBrakeThreadError, wheelThreadError;

   gasThreadError = pthread_create(&gasThread, NULL, gasListener, arg);
   handBrakeThreadError = pthread_create(&handBrakeThread, NULL, handBrakeListener, arg);
   wheelThreadError = pthread_create(&wheelThread, NULL, wheelListener, arg);

   pthread_join(gasThread, NULL);
   pthread_join(handBrakeThread, NULL);
   pthread_join(wheelThread, NULL);

   printf("Gas thread returned: %d\n", gasThreadError);
   printf("Hand Brake thread returned: %d\n", handBrakeThreadError);
   printf("Wheel thread returned: %d\n", wheelThreadError);
}

void *keyListener(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;
   ros::init(container->argc, container->argv, "keyListener");
   ros::NodeHandle handle;
   ros::Subscriber subscriber = handle.subscribe("drc_vehicle_xp900/key/state", 1000, chatterCallback2);
   ros::spin();
   return ptr;
}

void *wheelListener(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;
   ros::init(container->argc, container->argv, "wheelListener");
   ros::NodeHandle handle;
   ros::Subscriber subscriber = handle.subscribe("drc_vehicle_xp900/hand_wheel/state", 1000, chatterCallback);
   ros::spin();
   return ptr;
}
void *gasListener(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;
   ros::init(container->argc, container->argv, "wheelListener");
   ros::NodeHandle handle;
   ros::Subscriber subscriber = handle.subscribe("drc_vehicle_xp900/hand_wheel/state", 1000, chatterCallback);
   ros::spin();
   return ptr;
}

void *handBrakeListener(void *ptr)
{
   inputContainer* container = (inputContainer *) ptr;
   ros::init(container->argc, container->argv, "handBrakeListener");
   ros::NodeHandle handle;
   ros::Subscriber subscriber = handle.subscribe("drc_vehicle_xp900/hand_brake/state", 1000, chatterCallback);
   ros::spin();
   return ptr;
}

void chatterCallback2(const std_msgs::Int8::ConstPtr& msg)
{
   ROS_INFO("I heard: [%d]:", msg->data);
}
