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

#include "base_wrapper.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

class StereoWrapper : public BaseWrapper
{
public:
  /**
   * @brief StereoWrapper c'tor
   * @param sensor
   * @param nh
   */
  StereoWrapper(const ORB_SLAM3::System::eSensor& sensor, ros::NodeHandle& nh);

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
