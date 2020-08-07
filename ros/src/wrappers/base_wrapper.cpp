#include "base_wrapper.h"

namespace orb_slam3_ros
{
BaseWrapper::BaseWrapper(const ORB_SLAM3::System::eSensor &sensor, ros::NodeHandle &nh,
                         image_transport::ImageTransport &it)
  : nh_(nh)
{
  // read input parameters
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("vocabulary_filename", vocabulary_filename_, "");
  pnh.param<std::string>("settings_filename", settings_filename_, "");
  pnh.param<std::string>("map_frame_name", map_frame_name_, "map");
  pnh.param<std::string>("camera_frame_name", camera_frame_name_, "camera_link");

  // instantiate orb slam system
  system_ = std::make_shared<ORB_SLAM3::System>(vocabulary_filename_, settings_filename_, sensor);

  // advertise pose publisher
  pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);

  // advertise current frame publisher
  image_publisher_ = it.advertise("/current_frame", 1);

  // advertise map publisher
  map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_points", 1);

  // init optical offset
  optical_offset_.setIdentity();
  optical_offset_.setBasis(tf2::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0));
}

BaseWrapper::~BaseWrapper()
{
  system_->Shutdown();
}

void BaseWrapper::publishCameraPose(const cv::Mat &cv_mat)
{
  // convert cv::Mat camera pose to tf2 Transform
  tf2::Transform camera_tf = totfTransform(cv_mat);

  // apply conversion between camera and optical frame
  camera_tf = optical_offset_ * camera_tf;
  camera_tf = camera_tf.inverse();
  camera_tf = optical_offset_ * camera_tf;

  // convert to pose msg
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = last_stamp_;
  pose_msg.header.frame_id = map_frame_name_;

  pose_msg.pose.position.x = camera_tf.getOrigin().getX();
  pose_msg.pose.position.y = camera_tf.getOrigin().getY();
  pose_msg.pose.position.z = camera_tf.getOrigin().getZ();

  pose_msg.pose.orientation.x = camera_tf.getRotation().getX();
  pose_msg.pose.orientation.y = camera_tf.getRotation().getY();
  pose_msg.pose.orientation.z = camera_tf.getRotation().getZ();
  pose_msg.pose.orientation.w = camera_tf.getRotation().getW();

  // publish
  pose_publisher_.publish(pose_msg);
}

void BaseWrapper::publishCameraTf(const cv::Mat &cv_mat)
{
  // convert cv::Mat camera pose to tf2 Transform
  tf2::Transform camera_tf = totfTransform(cv_mat);

  // apply coordinate conversion
  camera_tf = optical_offset_ * camera_tf;
  camera_tf = camera_tf.inverse();
  camera_tf = optical_offset_ * camera_tf;

  // publish tf with broadcaster
  geometry_msgs::TransformStamped camera_to_map_transform;
  camera_to_map_transform.header.stamp = last_stamp_;
  camera_to_map_transform.header.frame_id = map_frame_name_;
  camera_to_map_transform.child_frame_id = camera_frame_name_;
  tf2::convert(camera_tf, camera_to_map_transform.transform);
  tf_broadcaster_.sendTransform(camera_to_map_transform);
}

void BaseWrapper::publishCurrentFrame(const cv::Mat &frame)
{
  std_msgs::Header header;
  header.stamp = last_stamp_;
  header.frame_id = map_frame_name_;
  image_publisher_.publish(cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
}

void BaseWrapper::publishMap(std::vector<ORB_SLAM3::MapPoint *> map_points)
{
  if (map_points.size() == 0)
  {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;
  const int num_channels = 3;  // x y z

  cloud.header.stamp = last_stamp_;
  cloud.header.frame_id = map_frame_name_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z" };
  for (int i = 0; i < num_channels; i++)
  {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  for (unsigned int i = 0; i < cloud.width; i++)
  {
    if (map_points.at(i)->nObs >= min_observations_per_point_)
    {
      data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2);
      data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0);
      data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1);

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }

  map_publisher_.publish(cloud);
}

tf2::Transform BaseWrapper::totfTransform(const cv::Mat &cv_mat)
{
  tf2::Vector3 t = orb_slam3_ros::Converter::toTfVector3(cv_mat.rowRange(0, 3).col(3));
  tf2::Matrix3x3 R =
      orb_slam3_ros::Converter::toTfMatrix3x3(cv_mat.rowRange(0, 3).colRange(0, 3));
  return tf2::Transform(R, t);
}
}
