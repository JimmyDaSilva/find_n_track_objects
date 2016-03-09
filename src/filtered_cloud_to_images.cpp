#include <find_n_track_objects/filtered_cloud_to_images.hpp>

void kinectCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud, const sensor_msgs::Image::ConstPtr &ros_depth_image){
  
  // Convert to pcl cloud
  pcl::fromROSMsg(*ros_cloud, *pcl_cloud_);
  
  // Keep only the point inside the box
  cropFilter_.setInputCloud (pcl_cloud_); 
  cropFilter_.filter (*pcl_cloud_);
  
  // Set the rgb black for the outside points
  uint8_t r = 0, g = 0, b = 0;
  int32_t rgb = (r << 16) | (g << 8) | b; 
  for (size_t i = 0; i < pcl_cloud_->points.size (); ++i){
    if(isnan(pcl_cloud_->points[i].z))
      pcl_cloud_->points[i].rgb = rgb;
  }
  
  // Convert cloud to rgb ros image
  pcl::toROSMsg(*pcl_cloud_, *rgb_image_);
  rgb_image_->header.frame_id = inputframe_;
  pub_rgb_image_.publish(*rgb_image_);

  // Convert rgb image to a binary mask 
  cv_rgb_mask_ = cv_bridge::toCvCopy(rgb_image_, sensor_msgs::image_encodings::MONO8);  
  
  // Apply mask to depth image
  cv_depth_image_ = cv_bridge::toCvCopy(ros_depth_image);
  cv_depth_image_->image.copyTo(cv_depth_image_->image , cv_rgb_mask_->image);
  
  // Convert depth to ros image
  cv_depth_image_->toImageMsg(*final_depth_image_);
  pub_depth_image_.publish(*final_depth_image_);
}

int main(int argc, char **argv)
{
  // ROS initialization
  ros::init(argc, argv, "FilteredCloudToImages");
  ros::NodeHandle nh, nh_priv("~");
  
  // ApproximateTime synchronized callback of point cloud and depth image
  message_filters::Subscriber<sensor_msgs::PointCloud2> first_sub(nh, "/kinect1/depth_registered/points", 1);
  message_filters::Subscriber<sensor_msgs::Image> second_sub(nh, "/kinect1/depth_registered/image_raw", 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), first_sub, second_sub);
  sync.registerCallback(boost::bind(&kinectCallback, _1, _2));
  
  // ROS publishers for the filtered depth and rgb image
  pub_depth_image_ = nh.advertise<sensor_msgs::Image> ("/kinect1/depth_registered/filtered_depth_image", 1);
  pub_rgb_image_ = nh.advertise<sensor_msgs::Image> ("/kinect1/depth_registered/filtered_rgb_image", 1);
  
  // Pointcloud and images pointer initialization
  pcl_cloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
  final_depth_image_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
  rgb_image_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
  
  // Load params
  bool params_loaded = true;
//   params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);

  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    return -1;
  }
  
  // Compute box size, translation and rotation and size
  geometry_msgs::PointStamped min_point;
  min_point.header.frame_id = outframe_;
  min_point.header.stamp = ros::Time();
  min_point.point.x = 0.40;
  min_point.point.y = -0.25;
  geometry_msgs::PointStamped max_point;
  max_point.header.frame_id = outframe_;
  max_point.header.stamp = ros::Time();
  max_point.point.x = 0.90;
  max_point.point.y = 0.5;
  
  double box_height = 0.5;
  double box_width = (max_point.point.x - min_point.point.x);
  double box_depth = (max_point.point.y - min_point.point.y);
  
  Eigen::Vector4f box_min_point, box_max_point;
  Eigen::Vector3f box_trans, box_rot;
  
  box_min_point << -box_width/2.0, -box_depth/2.0, -box_height/2.0, 1.0;
  box_max_point << box_width/2.0, box_depth/2.0, box_height/2.0, 1.0;
  
  geometry_msgs::PoseStamped pose_box_center;
  pose_box_center.header.frame_id = outframe_;
  pose_box_center.pose.position.x = (min_point.point.x + max_point.point.x) / 2.0;
  pose_box_center.pose.position.y = (min_point.point.y + max_point.point.y) / 2.0;
  pose_box_center.pose.position.z = box_height / 2.0 - 0.04;
  pose_box_center.pose.orientation.w = 1.0;
  
  tf::TransformListener trans_listener;
  double roll, pitch, yaw;
  try{
    trans_listener.waitForTransform(outframe_, inputframe_, ros::Time(0), ros::Duration(10.0));   
    trans_listener.transformPose(inputframe_, pose_box_center, pose_box_center);
    tf::Quaternion quat(pose_box_center.pose.orientation.x, pose_box_center.pose.orientation.y, pose_box_center.pose.orientation.z, pose_box_center.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    box_rot << roll, pitch, yaw;
    box_trans << pose_box_center.pose.position.x, pose_box_center.pose.position.y, pose_box_center.pose.position.z;
  }
  catch(tf::TransformException &ex){
    std::cout << ex.what() << std::endl;
    ROS_ERROR("%s", ex.what());
    return -1;
  }
  
  // Configure cropBox filter
  cropFilter_.setKeepOrganized(true);
  cropFilter_.setMin(box_min_point); 
  cropFilter_.setMax(box_max_point); 
  cropFilter_.setTranslation(box_trans); 
  cropFilter_.setRotation(box_rot);
  cropFilter_.setNegative(false);  

  ros::spin();  
  return 0;
}
