#include <serial_to_can/topic_to_serial.h>

namespace serial_to_can
{
  TopicToSerial::TopicToSerial(ros::NodeHandle* nh, ros::NodeHandle* nh_param, RTCanSharedPtr driver)
  {
    can_topic_ = nh->subscribe<can_msgs::Frame>("sent_messages", 10, boost::bind(&TopicToSerial::msgCallback, this, _1));
    driver_ = driver;
  };

  void TopicToSerial::msgCallback(const can_msgs::Frame::ConstPtr& msg)
  {
    can_msgs::Frame m = *msg.get();
    CAN_msg f;
    
    convertMessageToSerial(m, f);
    bool res = driver_->SendCANMessage(&f, 0);
    if(!res) 
    {
        ROS_ERROR("Failed to send message");
    }
  };
  
  void convertMessageToSerial(const can_msgs::Frame& m, CAN_msg &f)
  {
    f.id = m.id;
    f.len = m.dlc;
    f.format = m.is_extended ? EXTENDED_FORMAT : STANDARD_FORMAT;
    f.type = m.is_rtr ? REMOTE_FRAME : DATA_FRAME;
    f.ch = 0x00; // TODO: 现只支持数据包
    for( int i = 0; i < 8; i++)
    {
        f.data[i] = m.data[i];
    }
  };
}; // namespace name
