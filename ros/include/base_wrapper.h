#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>

#include "System.h"
#include "converter.h"

namespace orb_slam3_ros
{
class BaseWrapper
{
public:
  /**
   * @brief BaseWrapper c'tor
   * @param sensor
   * @param nh
   */
  BaseWrapper(const ORB_SLAM3::System::eSensor& sensor, ros::NodeHandle& nh,
              image_transport::ImageTransport& it);


  /**
   * @brief BaseWrapper d'tor
   */ ~BaseWrapper();

  /**
   * @brief publishCameraPose
   * @param cv_mat
   */
  void publishCameraPose(const cv::Mat& cv_mat);

  /**
   * @brief publishCameraTf
   * @param cv_mat
   */
  void publishCameraTf(const cv::Mat& cv_mat);

  /**
   * @brief publishCurrentFrame
   * @param frame
   */
  void publishCurrentFrame(const cv::Mat& frame);

  /**
   * @brief publishMap
   * @param map_points
   */
  void publishMap(std::vector<ORB_SLAM3::MapPoint*> map_points);


protected:
  // ros node handle
  ros::NodeHandle nh_;

  // camera_pose publisher
  ros::Publisher pose_publisher_;

  // tf2 broadcaster for publishing camera tf
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // current frame publisher
  image_transport::Publisher image_publisher_;

  // map publisher
  ros::Publisher map_publisher_;

  // orb slam system (to perform mapping and localization)
  std::shared_ptr<ORB_SLAM3::System> system_;
  std::string vocabulary_filename_;
  std::string settings_filename_;

  // params
  double min_observations_per_point_ = 5;
  tf2::Transform optical_offset_;

  // info for message headers
  ros::Time last_stamp_;
  std::string map_frame_name_;
  std::string camera_frame_name_;

  // utility function for converting cv::Mat to tf2::Transform
  tf2::Transform totfTransform(const cv::Mat& cv_mat);
};
}
