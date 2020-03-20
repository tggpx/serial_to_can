#include <serial_to_can/serial_to_topic.h>

namespace serial_to_can
{

  SerialToTopic::SerialToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param, RTCanSharedPtr driver)
  {
        can_topic_ = nh->advertise<can_msgs::Frame>("received_messages", 10);
        driver_ = driver;
  };

  void SerialToTopic::setup()
  {
    driver_->SetRxPackageCallBack(RxCallBack, this);
  };

  void SerialToTopic::frameCallback(const CAN_msg& f)
  {
    can_msgs::Frame msg;
    converSerialToMessage(f, msg);

    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    can_topic_.publish(msg);
  };

  void SerialToTopic::RxCallBack(void *pParam, const BYTE * byBuf, DWORD dwLen)
  {
    CAN_msg *f = (CAN_msg *)byBuf;
    ((SerialToTopic *)pParam)->frameCallback(*f);
  };

  void converSerialToMessage(const CAN_msg& f, can_msgs::Frame& m)
  {
    m.id = f.id;
    m.dlc = f.len;
    m.is_extended = (EXTENDED_FORMAT == f.format)? true : false;
    m.is_rtr = (REMOTE_FRAME == f.type) ? true : false;
    m.is_error = false; //TODO: 暂不支持错误帧处理
    for(int i = 0; i < 8; i++)
    {
      m.data[i] = f.data[i];
    }
  };

}
