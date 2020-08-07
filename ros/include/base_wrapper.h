/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José
* M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University
* of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of
* the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License,
* or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

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
