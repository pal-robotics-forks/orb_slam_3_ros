#include "stereo_wrapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_slam_node");
  ros::start();

  ros::NodeHandle nh;

  StereoWrapper wrapper(ORB_SLAM3::System::STEREO, nh);

  ros::spin();

  return 0;
}
