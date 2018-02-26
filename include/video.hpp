#ifndef VIDEO_HPP
#define VIDEO_HPP

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

using namespace std;

class Video{
public:
  Video(string str);
  void recSetup();
  void recCycle(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
  void recEnd();
private:
  int flg;
  string name;
  ofstream fout;
  struct timeval recordTime;
  time_t old_sec;
  suseconds_t old_usec;
  size_t frame;
  std::ostringstream oss;
  pcl::PCDWriter writer;
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  string filePass;
};

Video::Video(string str){
  flg = 0;
  frame = 0;
  filePass = str;
}

void Video::recSetup(){
  flg = 0;
  name = filePass;
  time_t now = time(NULL);
  struct tm *tm_now  = localtime(&now);
  name += std::to_string(tm_now->tm_year + 1900);
  name += ".";
  name += std::to_string(tm_now->tm_mon + 1);
  name += ".";
  name += std::to_string(tm_now->tm_mday);
  name += "_";
  name += std::to_string(tm_now->tm_hour);
  name += ":";
  name += std::to_string(tm_now->tm_min);
  name += ":";
  name += std::to_string(tm_now->tm_sec);
  mkdir(name.c_str(), 0775);
  fout.open(name + "/timeStamp.txt");
  gettimeofday(&recordTime, NULL);
  old_sec = recordTime.tv_sec;
  old_usec = recordTime.tv_usec;
  frame = 0;
}

void Video::recCycle(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  oss.str("");
  oss << "./" << name << "/" << std::setfill('0') << std::setw(4) << frame;
  const std::string baseName = oss.str();
  const std::string cloudName = baseName + "_cloud.pcd";
  OUT_INFO("saving cloud: " << cloudName);
  writer.writeBinary(cloudName, *cloud);
  //OUT_INFO("saving complete!");
  gettimeofday(&recordTime, NULL);
  fout << std::setfill('0') << std::setw(4) << frame;
  fout << "_cloud.pcd" << endl;
  fout << std::to_string((recordTime.tv_sec - old_sec)+ (recordTime.tv_usec - old_usec)/1000.0/1000.0) << endl;
  ++frame;
}

void Video::recEnd(){
  fout.close();
}

#endif // VIDEO_HPP
