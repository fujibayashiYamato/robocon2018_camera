#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "util.hpp"

class RosSerial{
public:
  RosSerial();
  void messgeCallback(const std_msgs::Float32MultiArray::ConstPtr& rx_msg);
  void write(float y,float z);
  void cycle();
  void getPos(Coord<float,float> &coord);
  bool dataAvailable();
private:
  Coord<float,float> coord;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  std_msgs::Float32MultiArray tx_msg;
  bool availableFlg;
};

RosSerial::RosSerial()
{
  sub = nh.subscribe("chatter",1000,&RosSerial::messgeCallback, this);
  pub = nh.advertise<std_msgs::Float32MultiArray>("array", 1000);
  availableFlg = false;
}

void RosSerial::messgeCallback(const std_msgs::Float32MultiArray::ConstPtr& rx_msg)
{
  //printf("%f:%f:%f\n",rx_msg->data[0],rx_msg->data[1],rx_msg->data[2]);
  coord.cartesianX(rx_msg->data[0]);
  coord.cartesianY(rx_msg->data[1]);
  coord.angleZ(rx_msg->data[2]);
  availableFlg = true;
}

void RosSerial::write(float y,float z)
{
  tx_msg.data.clear();
  tx_msg.data.push_back(y);
  tx_msg.data.push_back(z);
  pub.publish(tx_msg);
}

void RosSerial::getPos(Coord<float,float> &coord)
{
  coord = this->coord;
  availableFlg = false;
}

bool RosSerial::dataAvailable(){return availableFlg;}

void RosSerial::cycle()
{
  ros::spinOnce();
}

#endif // SERIAL_HPP
