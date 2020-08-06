#include "base_wrapper.h"


BaseWrapper::BaseWrapper(const ORB_SLAM3::System::eSensor &sensor, ros::NodeHandle &nh)
  : nh_(nh)
{
  // read input parameters
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("vocabulary_filename", vocabulary_filename_, "");
  pnh.param<std::string>("settings_filename", settings_filename_, "");

  // instantiate orb slam system
  system_ = std::make_shared<ORB_SLAM3::System>(vocabulary_filename_, settings_filename_, sensor);
}

BaseWrapper::~BaseWrapper()
{
  system_->Shutdown();
}
