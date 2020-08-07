#include "converter.h"

namespace orb_slam3_ros
{
tf2::Vector3 Converter::toTfVector3(const cv::Mat &cv_vector)
{
  return tf2::Vector3(cv_vector.at<float>(0), cv_vector.at<float>(1), cv_vector.at<float>(2));
}

tf2::Matrix3x3 Converter::toTfMatrix3x3(const cv::Mat &cv_mat)
{
  return tf2::Matrix3x3(cv_mat.at<float>(0, 0), cv_mat.at<float>(0, 1),
                        cv_mat.at<float>(0, 2), cv_mat.at<float>(1, 0),
                        cv_mat.at<float>(1, 1), cv_mat.at<float>(1, 2), cv_mat.at<float>(2, 0),
                        cv_mat.at<float>(2, 1), cv_mat.at<float>(2, 2));
}
}
