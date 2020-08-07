#include "stereo_wrapper.h"

using namespace orb_slam3_ros;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_slam_node");
  ros::start();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  StereoWrapper wrapper(ORB_SLAM3::System::STEREO, nh, it);

  ros::spin();

  return 0;
}
