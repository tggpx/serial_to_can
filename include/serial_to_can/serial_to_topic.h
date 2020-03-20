#ifndef SERIAL_TO_TOPIC_H
#define SERIAL_TO_TOPIC_H

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <serial_to_can/RT_CAN.h>

namespace serial_to_can
{
class SerialToTopic
{
  public:
    SerialToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param, RTCanSharedPtr driver);
    void setup();
  private:
    ros::Publisher can_topic_;
    RTCanSharedPtr driver_;
    void frameCallback(const CAN_msg& f);
    static void RxCallBack(void *pParam, const BYTE * byBuf, DWORD dwLen);
};

void converSerialToMessage(const CAN_msg& f, can_msgs::Frame& m);

};
#endif