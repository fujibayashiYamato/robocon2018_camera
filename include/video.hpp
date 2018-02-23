#ifndef VIDEO_HPP
#define VIDEO_HPP

class Video{
public:
  Video();
  void setup();
  void cycle();
private:
  int flg;
};

Video::Video(){
  flg = 0;
}

void Video::setup(){
  flg = 0;
}

void Video::cycle(){
  if(flg >= 1){
    if(flg == 1){
      flg = 2;
      name = "src/iai_kinect2/kinect2_viewer/pcd/";
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
    }
  }
}

#endif // VIDEO_HPP
