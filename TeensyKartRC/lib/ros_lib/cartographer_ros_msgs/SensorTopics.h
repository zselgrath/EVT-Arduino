#ifndef _ROS_cartographer_ros_msgs_SensorTopics_h
#define _ROS_cartographer_ros_msgs_SensorTopics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

  class SensorTopics : public ros::Msg
  {
    public:
      typedef const char* _laser_scan_topic_type;
      _laser_scan_topic_type laser_scan_topic;
      typedef const char* _multi_echo_laser_scan_topic_type;
      _multi_echo_laser_scan_topic_type multi_echo_laser_scan_topic;
      typedef const char* _point_cloud2_topic_type;
      _point_cloud2_topic_type point_cloud2_topic;
      typedef const char* _imu_topic_type;
      _imu_topic_type imu_topic;
      typedef const char* _odometry_topic_type;
      _odometry_topic_type odometry_topic;
      typedef const char* _nav_sat_fix_topic_type;
      _nav_sat_fix_topic_type nav_sat_fix_topic;
      typedef const char* _landmark_topic_type;
      _landmark_topic_type landmark_topic;

    SensorTopics():
      laser_scan_topic(""),
      multi_echo_laser_scan_topic(""),
      point_cloud2_topic(""),
      imu_topic(""),
      odometry_topic(""),
      nav_sat_fix_topic(""),
      landmark_topic("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_laser_scan_topic = strlen(this->laser_scan_topic);
      varToArr(outbuffer + offset, length_laser_scan_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->laser_scan_topic, length_laser_scan_topic);
      offset += length_laser_scan_topic;
      uint32_t length_multi_echo_laser_scan_topic = strlen(this->multi_echo_laser_scan_topic);
      varToArr(outbuffer + offset, length_multi_echo_laser_scan_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->multi_echo_laser_scan_topic, length_multi_echo_laser_scan_topic);
      offset += length_multi_echo_laser_scan_topic;
      uint32_t length_point_cloud2_topic = strlen(this->point_cloud2_topic);
      varToArr(outbuffer + offset, length_point_cloud2_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->point_cloud2_topic, length_point_cloud2_topic);
      offset += length_point_cloud2_topic;
      uint32_t length_imu_topic = strlen(this->imu_topic);
      varToArr(outbuffer + offset, length_imu_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->imu_topic, length_imu_topic);
      offset += length_imu_topic;
      uint32_t length_odometry_topic = strlen(this->odometry_topic);
      varToArr(outbuffer + offset, length_odometry_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->odometry_topic, length_odometry_topic);
      offset += length_odometry_topic;
      uint32_t length_nav_sat_fix_topic = strlen(this->nav_sat_fix_topic);
      varToArr(outbuffer + offset, length_nav_sat_fix_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->nav_sat_fix_topic, length_nav_sat_fix_topic);
      offset += length_nav_sat_fix_topic;
      uint32_t length_landmark_topic = strlen(this->landmark_topic);
      varToArr(outbuffer + offset, length_landmark_topic);
      offset += 4;
      memcpy(outbuffer + offset, this->landmark_topic, length_landmark_topic);
      offset += length_landmark_topic;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_laser_scan_topic;
      arrToVar(length_laser_scan_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_laser_scan_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_laser_scan_topic-1]=0;
      this->laser_scan_topic = (char *)(inbuffer + offset-1);
      offset += length_laser_scan_topic;
      uint32_t length_multi_echo_laser_scan_topic;
      arrToVar(length_multi_echo_laser_scan_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_multi_echo_laser_scan_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_multi_echo_laser_scan_topic-1]=0;
      this->multi_echo_laser_scan_topic = (char *)(inbuffer + offset-1);
      offset += length_multi_echo_laser_scan_topic;
      uint32_t length_point_cloud2_topic;
      arrToVar(length_point_cloud2_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_point_cloud2_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_point_cloud2_topic-1]=0;
      this->point_cloud2_topic = (char *)(inbuffer + offset-1);
      offset += length_point_cloud2_topic;
      uint32_t length_imu_topic;
      arrToVar(length_imu_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_imu_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_imu_topic-1]=0;
      this->imu_topic = (char *)(inbuffer + offset-1);
      offset += length_imu_topic;
      uint32_t length_odometry_topic;
      arrToVar(length_odometry_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_odometry_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_odometry_topic-1]=0;
      this->odometry_topic = (char *)(inbuffer + offset-1);
      offset += length_odometry_topic;
      uint32_t length_nav_sat_fix_topic;
      arrToVar(length_nav_sat_fix_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nav_sat_fix_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nav_sat_fix_topic-1]=0;
      this->nav_sat_fix_topic = (char *)(inbuffer + offset-1);
      offset += length_nav_sat_fix_topic;
      uint32_t length_landmark_topic;
      arrToVar(length_landmark_topic, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_landmark_topic; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_landmark_topic-1]=0;
      this->landmark_topic = (char *)(inbuffer + offset-1);
      offset += length_landmark_topic;
     return offset;
    }

    const char * getType(){ return "cartographer_ros_msgs/SensorTopics"; };
    const char * getMD5(){ return "bafbff7d66e3eeeb8d4a9195096cba08"; };

  };

}
#endif