#ifndef TOPIC_TO_SERIAL_H
#define TOPIC_TO_SERIAL_H

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <serial_to_can/RT_CAN.h>

namespace serial_to_can
{
class TopicToSerial
{
    public:
      TopicToSerial(ros::NodeHandle* nh, ros::NodeHandle* nh_param, RTCanSharedPtr driver);
    private:
      ros::Subscriber can_topic_;
      RTCanSharedPtr driver_;

      void msgCallback(const can_msgs::Frame::ConstPtr& msg);
};

void convertMessageToSerial(const can_msgs::Frame& m, CAN_msg &f);
}; // namespace name



#endif