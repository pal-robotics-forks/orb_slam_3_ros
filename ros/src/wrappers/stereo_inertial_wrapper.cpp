#include "stereo_inertial_wrapper.h"

namespace orb_slam3_ros
{
StereoInertialWrapper::StereoInertialWrapper(const ORB_SLAM3::System::eSensor &sensor,
                                             ros::NodeHandle &nh,
                                             image_transport::ImageTransport &it)
  : BaseWrapper(sensor, nh, it)

{
  left_img_sub_ = nh.subscribe("image_left/image_color_rect", 1,
                               &StereoInertialWrapper::grabLeftImage, this);
  right_img_sub_ = nh.subscribe("image_right/image_color_rect", 1,
                                &StereoInertialWrapper::grabRightImage, this);

  imu_sub_ = nh.subscribe("imu", 1, &StereoInertialWrapper::grabImu, this);

  sync_timer_ =
      nh.createTimer(ros::Duration(0.01), &StereoInertialWrapper::synchronizeMessages, this);
}

void StereoInertialWrapper::grabLeftImage(const sensor_msgs::Image::ConstPtr &msg)
{
  // acquire left buffer lock
  boost::recursive_mutex::scoped_lock lock(left_lock_);

  // if not empty, remove oldest element
  if (!left_img_buffer_.empty())
  {
    left_img_buffer_.pop();
  }

  // insert new element
  left_img_buffer_.push(msg);
}

void StereoInertialWrapper::grabRightImage(const sensor_msgs::Image::ConstPtr &msg)
{
  // acquire right buffer lock
  boost::recursive_mutex::scoped_lock lock(right_lock_);

  // if not empty, remove oldest element
  if (!right_img_buffer_.empty())
  {
    right_img_buffer_.pop();
  }

  // insert new element
  right_img_buffer_.push(msg);
}

void StereoInertialWrapper::grabImu(const sensor_msgs::Imu::ConstPtr &msg)
{
  // acquire imu buffer lock
  boost::recursive_mutex::scoped_lock lock(imu_lock_);

  // insert new element
  imu_buffer_.push(msg);
}

void StereoInertialWrapper::synchronizeMessages(const ros::TimerEvent &)
{
  // check that buffers are not empty
  if (left_img_buffer_.empty() || right_img_buffer_.empty() || imu_buffer_.empty())
  {
    return;
  }

  double left_img_stamp = left_img_buffer_.front()->header.stamp.toSec();
  double right_img_stamp = right_img_buffer_.front()->header.stamp.toSec();

  // acquire left buffer lock
  {
    boost::recursive_mutex::scoped_lock lock(left_lock_);

    // discard old messages
    while ((right_img_stamp - left_img_stamp) > max_time_difference_ &&
           left_img_buffer_.size() > 1)
    {
      left_img_buffer_.pop();
      left_img_stamp = left_img_buffer_.front()->header.stamp.toSec();
    }
  }

  // acquire right buffer lock
  {
    boost::recursive_mutex::scoped_lock lock(right_lock_);

    // discard old messages
    while ((left_img_stamp - right_img_stamp) > max_time_difference_ &&
           right_img_buffer_.size() > 1)
    {
      right_img_buffer_.pop();
      right_img_stamp = right_img_buffer_.front()->header.stamp.toSec();
    }
  }

  // discard messages whose stamps are too far away
  if (std::abs(left_img_stamp - right_img_stamp) > max_time_difference_)
  {
    return;
  }

  // check that there is no new imu message
  if (left_img_stamp > imu_buffer_.back()->header.stamp.toSec())
  {
    return;
  }

  // acquire left buffer lock
  cv_bridge::CvImageConstPtr left_cv_ptr;
  {
    boost::recursive_mutex::scoped_lock lock(left_lock_);

    try
    {
      left_cv_ptr = cv_bridge::toCvShare(left_img_buffer_.front());
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // acquire right buffer lock
  cv_bridge::CvImageConstPtr right_cv_ptr;
  {
    boost::recursive_mutex::scoped_lock lock(right_lock_);

    try
    {
      right_cv_ptr = cv_bridge::toCvShare(right_img_buffer_.front());
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // acquire imu buffer lock
  std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
  {
    boost::recursive_mutex::scoped_lock lock(imu_lock_);

    // check that imu buffer is not empty
    if (imu_buffer_.empty())
    {
      return;
    }

    // retrieve last imu measurements
    while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp.toSec() < left_img_stamp)
    {
      double imu_stamp = imu_buffer_.front()->header.stamp.toSec();
      cv::Point3f acc(imu_buffer_.front()->linear_acceleration.x,
                      imu_buffer_.front()->linear_acceleration.y,
                      imu_buffer_.front()->linear_acceleration.z);
      cv::Point3f gyr(imu_buffer_.front()->angular_velocity.x,
                      imu_buffer_.front()->angular_velocity.y,
                      imu_buffer_.front()->angular_velocity.z);
      imu_measurements.push_back(ORB_SLAM3::IMU::Point(acc, gyr, imu_stamp));
      imu_buffer_.pop();
    }
  }

  // track camera pose with ORB_SLAM3
  cv::Mat camera_pose = system_->TrackStereo(left_cv_ptr->image, right_cv_ptr->image,
                                             left_img_stamp, imu_measurements);

  // when track lost -> do nothing
  if (camera_pose.empty())
  {
    return;
  }

  // store last stamp
  last_stamp_ = left_img_buffer_.front()->header.stamp;

  // publish estimated pose and tf
  publishCameraTf(camera_pose);
  publishCameraPose(camera_pose);

  // publish current frame
  publishCurrentFrame(system_->DrawCurrentFrame());

  // publish feature map
  publishMap(system_->GetAllMapPoints());
}
}
