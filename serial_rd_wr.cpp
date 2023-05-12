#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <iostream>

serial::Serial ser("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
ros::Publisher left_encoder_pub;
ros::Publisher right_encoder_pub;
int encoder_pub_rate = 10;
uint32_t t_prev = 0;

uint32_t millis()
{
  return ros::Time::now().toNSec() / 1000000;
}

void encoder_publisher(uint16_t enc1, uint16_t enc2)
{
  std_msgs::Float32 left_encoder_msg;
  left_encoder_msg.data = static_cast<float>(enc1);
  left_encoder_pub.publish(left_encoder_msg);
  std_msgs::Float32 right_encoder_msg;
  right_encoder_msg.data = static_cast<float>(enc2);
  right_encoder_pub.publish(right_encoder_msg);
  std::cout << "enc1: " << enc1 << ", enc2: " << enc2 << std::endl;  // print encoders to console
}

void read_serial()
{
  uint16_t enc1 = 0;
  uint16_t enc2 = 0;
  uint8_t x;
  ser.read(&x, 1);
  if (x == 0xaa)
  {
    uint8_t a[35];
    ser.read(a, 35);
    if (a[0] == 0x55)
    {
      enc2 = ((uint16_t)a[10] << 8) | (uint16_t)a[11];
      enc2 %= 65536;  // normalize enc2 to the range 0-65535
      enc1 = ((uint16_t)a[6] << 8) | (uint16_t)a[7];
      enc1 %= 65536;  // normalize enc1 to the range 0-65535
      if ((millis() - t_prev) >= (1000 / encoder_pub_rate))
      {
        encoder_publisher(enc1, enc2);
        t_prev = millis();
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serial_rd_wr");
  ros::NodeHandle nh;
  left_encoder_pub = nh.advertise<std_msgs::Float32>("/left_encoder", 1);
  right_encoder_pub = nh.advertise<std_msgs::Float32>("/right_encoder", 1);
  t_prev = millis();
  while (ros::ok())
  {
    read_serial();
    ros::spinOnce();
  }
  return 0;
}
