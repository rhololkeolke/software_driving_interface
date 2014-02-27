#ifndef SOFTWARE_DRIVING_INTERFACE_HH
#define SOFTWARE_DRIVING_INTERFACE_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>


namespace sdi
{
   class SoftwareDrivingInterface
   {
      public: SoftwareDrivingInterface();
      public: virtual ~SoftwareDrivingInterface();
      public: void Load();
      private: void RosPublishStates();
      private: void ConvertHDIMsgToSimMsgs();
      private: void GetHDIMsg();
      public: common::Time GetRosPublishPeriod();
      public: void SetRosPublishRate(double hz);

      private: event::ConnectionPtr ros_publish_connection_;
      private: ros::NodeHandle* rosNode;
      private: ros::CallbackQueue queue;
      private: void QueueThread();
      private: boost::thread callbackQueueThread;

      // Publishers
      private: ros::Publisher pubHDIFeedback;
      private: ros::Publisher pubGasPedalCmd;
      private: ros::Publisher pubBrakePedalCmd;
      private: ros::Publisher pubHandWheelCmd;
      private: ros::Publisher pubHandBrakeCmd;
      private: ros::Publisher pubKeyCmd;
      private: ros::Publisher pubDirectionCmd;

      // Subscribers
      private: ros::Subscriber subHDIControl;
      private: ros::Subscriber subGasPedalState;
      private: ros::Subscriber subBrakePedalState;
      private: ros::Subscriber subHandWheelState;
      private: ros::Subscriber subHandBrakeState;
      private: ros::Subscriber subKeyState;
      private: ros::Subscriber subDirectionState;

      // Ros timing
      private: common::Time rosPublishPeriod;
      private: common::Time lastRosPublishTime;
   }
}

#endif
