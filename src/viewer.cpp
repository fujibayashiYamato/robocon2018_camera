/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
 #include <math.h>

 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/ros/conversions.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/segmentation/sac_segmentation.h>

 #include <pcl/PolygonMesh.h>
 #include <pcl/io/vtk_lib_io.h>

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

#include "pcl_util.hpp"
#include "orbit.hpp"
#include "serial.hpp"
#include "util.hpp"


#include <pcl/filters/statistical_outlier_removal.h>

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH,
    PCL,
    DEBUG,
    CHECK
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case PCL:
      pclViewer();
      break;
    case DEBUG:
      debugViewer();
      break;
    case CHECK:
      checkViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  //#define DEBUG_DATA_ON
  #define DATA_VIEW_ON
  bool rec_flg = false;
  Coord<float,float> cameraSetupPos;
  float geinAir = 0.02;

  void pclViewer()
  {
    cv::Mat color, depth;
    #ifdef DATA_VIEW_ON
      pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
      const std::string cloudName = "rendered";
    #endif

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    #ifdef DATA_VIEW_ON
      visualizer->addPointCloud(cloud, cloudName);
      visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
      visualizer->initCameraParameters();
      visualizer->setBackgroundColor(0, 0, 0);
      visualizer->setPosition(mode == PCL ? color.cols : 0, 0);
      visualizer->setSize(color.cols, color.rows);
      visualizer->setShowFPS(true);
      visualizer->setCameraPosition(0,0,0,0,-1,0);
      visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);
    #endif

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointViewCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ringCutCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coordConversionCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsCutCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coat(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/ubuntu/catkin_ws/src/iai_kinect2/kinect2_viewer/pcd/coat01.pcd", *coat);

    Orbit orbit;
    RosSerial serial;

    bool start_flg = false;
    bool setup_flg = true;
    bool debug_flg = false;

    struct timeval recTime;
    time_t old_sec = 0;
    suseconds_t old_usec = 0;

    Coord<float,float> robotPos;
    robotPos.cartesianX(TZ3_X);
    robotPos.cartesianY(TZ3_Y+0.108);
    robotPos.angleZ(0.0 * M_PI/180.0);//0.0
    orbit.setInitRobotXYZ(robotPos);
    orbit.setRobotXYZ(robotPos);

    cameraSetupPos.cartesianX(0.0);
    cameraSetupPos.cartesianY(0.0);//0.13
    cameraSetupPos.cartesianZ(0.033);//0.04
    cameraSetupPos.angleY(-23.75 * M_PI/180.0);//-25.5
    orbit.setCameraSetupPos(cameraSetupPos);

    float xyz_centroid_buf[8][3] = {
      {-2.59120,-0.04350,2.39335},
      {-2.92735,-0.04878,2.49008},
      {-3.27120,-0.06753,2.56061},
      {-3.57458,-0.06975,2.55976},
      {-3.90198,-0.06814,2.54958},
      {-4.21703,-0.07187,2.49546},
      {-4.49903,-0.07859,2.38862},
      {-4.80053,-0.07273,2.29812}
    };

    orbit.setGeinAir(0.0245);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {



        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr shuttleDiscoveryCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr buf(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr campus(new pcl::PointCloud<pcl::PointXYZRGBA>);

        createCloud(depth, color, cloud);

        //--------------------------------------------------
        gettimeofday(&recTime, NULL);

        //ロボットの座標の取得を検知
        if(serial.dataAvailable()){
          serial.getPos(robotPos);
          start_flg = true;
        }

        //デバック用
        if(rec_flg){
          start_flg = true;
        }

        //受信した時一回だけ行われる
        if(start_flg && setup_flg){
          setup_flg = false;
          old_sec = recTime.tv_sec;
          old_usec = recTime.tv_usec;
          orbit.setup();
          orbit.setInitRobotXYZ(robotPos);
          #ifdef DATA_VIEW_ON
            printf("setup\n");
          #endif
        }

        orbit.filter(cloud,filterCloud);
        *campus = *filterCloud;
        orbit.coordConversion(filterCloud);
        orbit.ringCut(filterCloud,ringCutCloud);
        //*ringCutCloud = *filterCloud;


        if(start_flg){
          if(((recTime.tv_sec - old_sec) + (recTime.tv_usec - old_usec)/1000.0/1000.0 >= 2.0 && !rec_flg) || debug_flg){
            orbit.coeCreate();
            orbit.addPointView(buf);
            float shuttlePoint[2] = {0.0};
            #ifdef DATA_VIEW_ON
              printf("AREA:%d\n",orbit.checkArea());
              int passCheckMode = orbit.passCheck(buf,&shuttlePoint[0],&shuttlePoint[1]);
            #else
              int passCheckMode = orbit.passCheck(&shuttlePoint[0],&shuttlePoint[1]);
            #endif
            if(passCheckMode == 0){
              #ifdef DATA_VIEW_ON
                printf("Y:%3.5f Z::%3.5f\n\n",shuttlePoint[0],shuttlePoint[1]);
              #endif
              serial.write(shuttlePoint[0],shuttlePoint[1]);
            }else if(passCheckMode == 1){
              #ifdef DATA_VIEW_ON
              int cupCheckMode = orbit.cupCheck(buf);
              if(cupCheckMode == 1){
                printf("RONBAI\n\n");
              }else if(cupCheckMode == 0){
                printf("NO_CUP\n\n");
              }else{
                  printf("GOAL\n\n");
              }
              #endif
              serial.write(0.0,0.0);
            }
            *pointViewCloud = *buf;
            start_flg = false;
            setup_flg = true;
            debug_flg = false;
          }else{
            orbit.setRobotXYZ(robotPos);

          #ifdef DEBUG_DATA_ON
            for(int i = 0; i< 8;i++){orbit.addShuttlePoint(xyz_centroid_buf[i][0],xyz_centroid_buf[i][1],xyz_centroid_buf[i][2]);}
            debug_flg = true;
          #else
            #ifdef DATA_VIEW_ON
              orbit.shuttleDiscovery(ringCutCloud,shuttleDiscoveryCloud);
            #else
              orbit.shuttleDiscovery(ringCutCloud);
            #endif
          #endif
          }
        }
        //--------------------------------------------------
        #ifdef DATA_VIEW_ON

          //*campus = *cloud;
          orbit.coordConversion(campus);
          rotationZ(campus,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
          moveCloud(campus,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());

          mergeCloud(campus,shuttleDiscoveryCloud,campus);
          mergeCloud(campus,pointViewCloud,campus);

          //orbit.ringCutNotMove(coat,coat);
          mergeCloud(campus,coat,campus);
          //coatView(campus);
          //orbit.coatView(campus);
          visualizer->updatePointCloud(campus, cloudName);//*/
        #endif
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      #ifdef DATA_VIEW_ON
        visualizer->spinOnce(10);
      #endif
    }
    ros::spinOnce;
    #ifdef DATA_VIEW_ON
      visualizer->close();
    #endif
  }

  bool filter_flg = false;


  void debugViewer(){
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0,0,0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coat(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/ubuntu/catkin_ws/src/iai_kinect2/kinect2_viewer/pcd/coat01.pcd", *coat);

    RosSerial serial;
    Coord<float,float> robotPos;

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        if(serial.dataAvailable()){
          serial.getPos(robotPos);
          printf("posX:%3.5f posY:%3.5f posZ:%3.5f\n",robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());
          printf("angleX:%3.5f angleY:%3.5f angleZ:%3.5f\n",robotPos.angleX(),robotPos.angleY(),robotPos.angleZ());
          serial.write(robotPos.cartesianX(),robotPos.cartesianY());
        }

        visualizer->updatePointCloud(coat, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  int keyMode = 0;
  void checkViewer(){
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    Orbit orbit;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr removalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr campus(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ringCutCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coat(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/ubuntu/catkin_ws/src/iai_kinect2/kinect2_viewer/pcd/coat01.pcd", *coat);

    struct timeval recTime;
    time_t old_sec = 0;
    suseconds_t old_usec = 0;

    Coord<float,float> robotPos;
    robotPos.cartesianX(TZ3_X);
    robotPos.cartesianY(TZ3_Y+0.108);
    robotPos.angleZ(0.0);
    orbit.setInitRobotXYZ(robotPos);
    orbit.setRobotXYZ(robotPos);

    cameraSetupPos.cartesianX(0.0);
  	cameraSetupPos.cartesianY(0.0);
  	cameraSetupPos.cartesianZ(0.033);
  	cameraSetupPos.angleY(-23.75 * M_PI/180.0);
    cameraSetupPos.angleZ(0.0 * M_PI/180.0);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        orbit.setCameraSetupPos(cameraSetupPos);
        printf("posX:%3.5f posY:%3.5f posZ:%3.5f\n",cameraSetupPos.cartesianX(),cameraSetupPos.cartesianY(),cameraSetupPos.cartesianZ());
        printf("angX:%3.5f angY:%3.5f angZ:%3.5f\n",cameraSetupPos.angleX()*180.0/M_PI,cameraSetupPos.angleY()*180.0/M_PI,cameraSetupPos.angleZ()*180.0/M_PI);
        printf("keyMode:%d\n\n",keyMode);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr campus(new pcl::PointCloud<pcl::PointXYZRGBA>);
        mergeCloud(campus,cloud,campus);

        rotationX(campus,M_PI_2);
        rotationZ(campus,-1.0 * M_PI_2);
        rotationY(campus,cameraSetupPos.angleY());
        rotationZ(campus,-1.0 * cameraSetupPos.angleZ());
        moveCloud(campus,cameraSetupPos.cartesianX(),cameraSetupPos.cartesianY(),cameraSetupPos.cartesianZ());
        rotationZ(campus,-1.0 * robotPos.angleZ());//ロボットの回転の+と関数の回転の+の方向が逆のため、-1,0をかける
        moveCloud(campus,robotPos.cartesianX(),robotPos.cartesianY(),robotPos.cartesianZ());

        //coatView(campus);
        *campus += *coat;

        visualizer->updatePointCloud(campus, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case 'i':
        filter_flg = !filter_flg;
        rec_flg = !rec_flg;
        break;
      case 'w':
        cameraSetupPos.cartesianX(cameraSetupPos.cartesianX()+0.001);
        geinAir += 0.001;
        break;
      case 's':
        cameraSetupPos.cartesianX(cameraSetupPos.cartesianX()-0.001);
        geinAir -= 0.001;
        break;
      case 'a':
        cameraSetupPos.cartesianY(cameraSetupPos.cartesianY()+0.001);
        break;
      case 'd':
          cameraSetupPos.cartesianY(cameraSetupPos.cartesianY()-0.001);
        break;
      case 'z':
        cameraSetupPos.cartesianZ(cameraSetupPos.cartesianZ()+0.001);
        break;
      case 'x':
        cameraSetupPos.cartesianZ(cameraSetupPos.cartesianZ()-0.001);
        break;
      case 'e':
        keyMode++;
        if(keyMode == 3)keyMode = 0;
        break;
      case 'c':
        switch (keyMode) {
        case 0:
          cameraSetupPos.angleX(cameraSetupPos.angleX()+0.25 * M_PI/180.0);
          break;
        case 1:
          cameraSetupPos.angleY(cameraSetupPos.angleY()+0.25 * M_PI/180.0);
          break;
        case 2:
          cameraSetupPos.angleZ(cameraSetupPos.angleZ()+0.25 * M_PI/180.0);
          break;
        }
        break;
      case 'v':
        switch (keyMode) {
        case 0:
          cameraSetupPos.angleX(cameraSetupPos.angleX()-0.25 * M_PI/180.0);
          break;
        case 1:
          cameraSetupPos.angleY(cameraSetupPos.angleY()-0.25 * M_PI/180.0);
          break;
        case 2:
          cameraSetupPos.angleZ(cameraSetupPos.angleZ()-0.25 * M_PI/180.0);
          break;
        }
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving cloud: " << cloudName);
    writer.writeBinary(cloudName, *cloud);
    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"subscriber_node");

#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "pcl")
    {
      mode = Receiver::PCL;
    }
    else if(param == "debug")
    {
      mode = Receiver::DEBUG;
    }
    else if(param == "check")
    {
      mode = Receiver::CHECK;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
