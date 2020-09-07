#include "stereo_inertial_wrapper.h"

using namespace orb_slam3_ros;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_inertial_slam_node");
  ros::start();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  StereoInertialWrapper wrapper(ORB_SLAM3::System::STEREO, nh, it);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
