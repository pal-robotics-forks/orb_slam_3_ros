#include "stereo_wrapper.h"

StereoWrapper::StereoWrapper(const ORB_SLAM3::System::eSensor &sensor, ros::NodeHandle &nh)
  : BaseWrapper(sensor, nh)
  , left_sub_(nh, "image_left/image_color_rect", 1)
  , right_sub_(nh, "image_right/image_color_rect", 1)
  , sync_(sync_pol(10), left_sub_, right_sub_)
{
  // register message filter callback
  sync_.registerCallback(boost::bind(&StereoWrapper::imageCallback, this, _1, _2));
}

void StereoWrapper::imageCallback(const sensor_msgs::Image::ConstPtr &left_msg,
                                  const sensor_msgs::Image::ConstPtr &right_msg)
{
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvShare(left_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvShare(right_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  system_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
}
