#pragma once

#include "base_wrapper.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace orb_slam3_ros
{
class StereoWrapper : public BaseWrapper
{
public:
  /**
   * @brief StereoWrapper c'tor
   * @param sensor
   * @param nh
   */
  StereoWrapper(const ORB_SLAM3::System::eSensor& sensor, ros::NodeHandle& nh,
                image_transport::ImageTransport& it);

  /**
   * @brief imageCallback
   * @param left_msg
   * @param right_msg
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr& left_msg,
                     const sensor_msgs::Image::ConstPtr& right_msg);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Subscriber<sensor_msgs::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_sub_;
  message_filters::Synchronizer<sync_pol> sync_;
};
}
