#include <ros/ros.h>
#include <string>
#include <serial_to_can/RT_CAN.h>
#include <serial_to_can/topic_to_serial.h>
#include <serial_to_can/serial_to_topic.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_to_can");
    ros::NodeHandle nh(""), nh_param("~");

    std::string serial_device;
    nh_param.param<std::string>("serial_device", serial_device, "/dev/ttyUSB0");

    int32_t baud;
    nh_param.param<int32_t>("baud_rate", baud, 115200);

    RTCanSharedPtr driver = boost::make_shared<RT_CAN>();
    driver->Open(serial_device, baud);
    serial_to_can::TopicToSerial to_serial(&nh, &nh_param, driver);
    serial_to_can::SerialToTopic to_topic(&nh, &nh_param, driver);
    to_topic.setup();

    ros::spin();
    ros::waitForShutdown();
    return 0;
}