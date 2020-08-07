#pragma once

#include<opencv2/core/core.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace orb_slam3_ros
{
class Converter
{
public:
  static tf2::Vector3 toTfVector3(const cv::Mat& cv_vector);
  static tf2::Matrix3x3 toTfMatrix3x3(const cv::Mat &cv_mat);
};
}
