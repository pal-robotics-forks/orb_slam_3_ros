#pragma once

#include "base_wrapper.h"

#include <queue>
#include <boost/thread.hpp>

#include <sensor_msgs/Imu.h>

namespace orb_slam3_ros
{
class StereoInertialWrapper : public BaseWrapper
{
public:
  StereoInertialWrapper(const ORB_SLAM3::System::eSensor& sensor, ros::NodeHandle& nh,
                        image_transport::ImageTransport& it);

  void grabLeftImage(const sensor_msgs::Image::ConstPtr& msg);
  void grabRightImage(const sensor_msgs::Image::ConstPtr& msg);

  void grabImu(const sensor_msgs::Imu::ConstPtr& msg);

  void synchronizeMessages(const ros::TimerEvent&);

private:
  ros::Subscriber left_img_sub_, right_img_sub_;
  ros::Subscriber imu_sub_;

  ros::Timer sync_timer_;

  boost::recursive_mutex left_lock_, right_lock_;
  queue<sensor_msgs::Image::ConstPtr> left_img_buffer_, right_img_buffer_;

  boost::recursive_mutex imu_lock_;
  queue<sensor_msgs::Imu::ConstPtr> imu_buffer_;

  double max_time_difference_ = 0.01;
};
}
