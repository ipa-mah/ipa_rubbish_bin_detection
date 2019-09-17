#include "wastebasket_detection.hpp"
void RPYToPose(const double& roll, const double& pitch, const double& yaw, geometry_msgs::Pose& pose)
{
  ROS_DEBUG_STREAM("roll: " << roll << " pitch: " << pitch << " yaw: " << yaw);
  pose = geometry_msgs::Pose();
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  ROS_DEBUG_STREAM("quaternion: " << q);

  pose.orientation = q;
}

void RPYToRotation(const double& roll, const double& pitch, const double& yaw, Eigen::Matrix3d& rotationMatrix)
{
  ROS_DEBUG_STREAM("roll: " << roll << " pitch: " << pitch << " yaw: " << yaw);
  Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = roll_angle * yaw_angle * pitch_angle;

  rotationMatrix = q.matrix();
}
IpaWasteBasteDetection::IpaWasteBasteDetection(ros::NodeHandle node_handle):node_handle_(node_handle),it_(0)
{
  //PARAMS
  ros::NodeHandle pnh("~");
  std::cout << "\n========== ipa_waste_baste_detection Parameters ==========\n";
  pnh.param("z_min_val", z_min_val_, 0.3);
  std::cout << "z_min_val = " << z_min_val_ << std::endl;
  pnh.param("z_max_val", z_max_val_, 1.4);
  std::cout << "z_max_val = " << z_max_val_ << std::endl;
  pnh.param("floor_plane_inlier_distance", floor_plane_inlier_distance_, 0.02); // 2 cm
  std::cout << "floor_plane_inlier_distance = " << floor_plane_inlier_distance_ << std::endl;

  pnh.param("floor_search_iterations", floor_search_iterations_, 3);
  std::cout << "floor_search_iterations = " << floor_search_iterations_ << std::endl;
  pnh.param("min_plane_points", min_plane_points_, 0);
  std::cout << "IpaWasteBasteDetection: min_plane_points = " << min_plane_points_ << std::endl;


  pnh.param("cluster_tolerance", cluster_tolerance_, 0.03);
  std::cout << "IpaWasteBasteDetection: cluster_tolerance = " << cluster_tolerance_ << std::endl;
  pnh.param("min_cluster_size", min_cluster_size_, 10);
  std::cout << "IpaWasteBasteDetection: min_cluster_size = " << min_cluster_size_ << std::endl;


  pnh.param("cylinder_distace_threshold", cylinder_distace_threshold_, 0.4);
  std::cout << "cylinder_distace_threshold = " << cylinder_distace_threshold_ << std::endl;

  pnh.param("cylinder_radius", cylinder_radius_, 0.2);
  std::cout << "cylinder_radius = " << cylinder_radius_ << std::endl;

  pnh.param("check_ground_truth_cylinder_height",check_ground_truth_cylinder_height_,false);
  std::cout << "check_ground_truth_cylinder_height = " << check_ground_truth_cylinder_height_ << std::endl;

  pnh.param("ground_truth_cylinder_height",ground_truth_cylinder_height_,0.31);
  std::cout << "ground_truth_cylinder_height = " << ground_truth_cylinder_height_ << std::endl;

  pnh.param("use_direction_plane",use_direction_plane_,false);
  std::cout << "use_direction_plane = " << use_direction_plane_ << std::endl;

  pnh.param("use_remove_outliers",use_remove_outliers_,true);
  std::cout << "use_remove_outliers = " << use_remove_outliers_ << std::endl;


  pnh.param("num_grasping_points",num_grasping_points_,8);
  std::cout << "num_grasping_points = " << num_grasping_points_ << std::endl;




  pnh.param("focal_x",focal_x_,614.6654);
  std::cout << "focal_x = " << focal_x_ << std::endl;
  pnh.param("focal_y",focal_y_,614.5995);
  std::cout << "focal_y = " << focal_y_ << std::endl;
  pnh.param("c_x",c_x_,326.94885);
  std::cout << "c_x = " << c_x_ << std::endl;
  pnh.param("c_y",c_y_,232.761398);
  std::cout << "c_y = " << c_y_ << std::endl;
  //Need to change
  cam_params_<<focal_x_,0,c_x_,0,focal_y_,c_y_,0,0,1;




  ///DEBUG PARAMS
  pnh.param("publish_floor_plane", debug_["publish_floor_plane"], false);
  std::cout << "DEBUG_IpaWasteBasteDetection: publish_floor_plane = " << debug_["publish_floor_plane"] << std::endl;

  pnh.param("publish_cylinder_object", debug_["publish_cylinder_object"], false);
  std::cout << "DEBUG_IpaWasteBasteDetection: publish_cylinder_object = " << debug_["publish_cylinder_object"] << std::endl;

  pnh.param("show_cylinder_image", debug_["show_cylinder_image"], false);
  std::cout << "DEBUG_IpaWasteBasteDetection: show_cylinder_image = " << debug_["show_cylinder_image"] << std::endl;

  pnh.param("show_cylinder_marker", debug_["show_cylinder_marker"], false);
  std::cout << "DEBUG_IpaWasteBasteDetection: show_cylinder_marker = " << debug_["show_cylinder_marker"] << std::endl;





  // dynamic reconfigure
  dynamic_reconfigure::Server<ipa_rubbish_bin_detection::ipa_rubbish_bin_detectionConfig>::CallbackType dynamic_reconfigure_callback_type;
  dynamic_reconfigure_callback_type = boost::bind(&IpaWasteBasteDetection::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_type);



  //Subscribing point cloud from camera and find cylinder(remap the point cloud name).
  camera_depth_points_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 2, &IpaWasteBasteDetection::preprocessingCallback, this);

  //Publisher for plane segmented cloud.
  floor_plane_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/floor_plane", 1);
  //Publisher for cylinder segmented cloud
  cylinder_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/cylinder",1);
  //Publisher for cylinder parameters
  cylinder_params_pub_  = node_handle_.advertise<ipa_rubbish_bin_detection::CylinderParams>("/cylinder_object",1);

  //publisher for ros visualization
  marker_pub_  = node_handle_.advertise<visualization_msgs::Marker>("visualization_camera_cylinder_object", 1);

  it_ = new image_transport::ImageTransport(node_handle_);
  cylinder_detection_image_pub_ = it_->advertise("cylinder_detection_image", 1);


  std::cout << "Waste Baste Detection initialized." << std::endl;


}

IpaWasteBasteDetection::~IpaWasteBasteDetection()
{
  if (it_ != 0)
    delete it_;
}
void IpaWasteBasteDetection::dynamicReconfigureCallback(ipa_rubbish_bin_detection::ipa_rubbish_bin_detectionConfig &config, uint32_t level)
{



  floor_search_iterations_ = config.floor_search_iterations;
  floor_plane_inlier_distance_ = config.floor_plane_inlier_distance;
  min_plane_points_ = config.min_plane_points;
  z_min_val_ = config.z_min_val;
  z_max_val_ = config.z_max_val;
  cylinder_distace_threshold_ = config.cylinder_distace_threshold;
  cylinder_radius_ = config.cylinder_radius;
  cluster_tolerance_ = config.cluster_tolerance;
  debug_["show_cylinder_marker"] = config.show_cylinder_marker;
  debug_["show_cylinder_image"] = config.show_cylinder_image;
  num_grasping_points_ = config.num_grasping_points;
  use_remove_outliers_ = config.use_remove_outliers;
  std::cout << "\n========== ipa_waste_baste_detection Dynamic reconfigure ==========\n";
  std::cout << "  floor_search_iterations = " << floor_search_iterations_ << std::endl;
  std::cout << "  floor_plane_inlier_distance = " << floor_plane_inlier_distance_ << std::endl;

  std::cout << "  cluster_tolerance = " << cluster_tolerance_ << std::endl;

  std::cout << "  cylinder_distace_threshold = " << cylinder_distace_threshold_ << std::endl;
  std::cout << "  cylinder_radius = " << cylinder_radius_ << std::endl;

  std::cout << "  min_plane_points = " << min_plane_points_ << std::endl;
  std::cout << "  z_min_val = " << z_min_val_ << std::endl;
  std::cout << "  z_max_val = " << z_max_val_ << std::endl;
  std::cout << "  use_remove_outliers = " << use_remove_outliers_ << std::endl;

  std::cout << "  num_grasping_points = " << num_grasping_points_ << std::endl;

  std::cout << "  show_cylinder_marker = " << debug_["show_cylinder_marker"] << std::endl;
  std::cout << "  show_cylinder_image = " << debug_["show_cylinder_image"] << std::endl;


}


void IpaWasteBasteDetection::preprocessingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{

  // get transform between camera and plane
  Eigen::Affine3d transform_cam2plane = Eigen::Affine3d::Identity();


  // convert point cloud message
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()), cylinder_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(*point_cloud2_rgb_msg, *input_cloud); //conversion Ros message->Pcl point cloud


  pcl::ModelCoefficients::Ptr cylinder_model (new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr plane_model (new pcl::ModelCoefficients);

  //parameters of the cylinder
  CylinderParams cylinder_params;
  cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
  if(debug_["show_cylinder_image"] == true)
  {
    int index = 0;
    for (std::size_t v = 0; v < input_cloud->height; v++)
    {
      for (std::size_t u = 0; u < input_cloud->width; u++, index++)
      {
        pcl::PointXYZRGB& point = (*input_cloud)[index];
        color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
      }
    }
  }
  bool found_cylinder = cylinderDetection(input_cloud,cylinder_cloud,color_image, point_cloud2_rgb_msg->header,
                                          plane_model,cylinder_model, transform_cam2plane,cylinder_params);

  if (!found_cylinder)
  {
    return;
  }
  ROS_INFO("Found cylinder object");
  //only visualize of found cylinder object
  if(debug_["show_cylinder_image"] == true)
  {
    cv_bridge::CvImage cv_ptr;
    cv_ptr.image = color_image;
    cv_ptr.encoding = "bgr8";
    cylinder_detection_image_pub_.publish(cv_ptr.toImageMsg());
  }







  //transform cylinder_cloud back to camera coordinate
  pcl::transformPointCloud (*cylinder_cloud, *cylinder_cloud, transform_cam2plane.inverse());
  //Publish TF transform of the center of cylinder object
  Eigen::Affine3d center_wtf_camera_pose = Eigen::Affine3d::Identity();

  // Computing and setting quaternion from axis angle representation.
  Eigen::Vector3d origin_z_direction(0., 0., 1.);
  Eigen::Vector3d axis_camera_cylinder;
  axis_camera_cylinder = origin_z_direction.cross(cylinder_params.direction_vec).normalized();
  double angle = std::acos(cylinder_params.direction_vec.dot(origin_z_direction));
  double theta = M_PI - std::acos(cylinder_params.direction_vec.dot(origin_z_direction));

  Eigen::Matrix3d plane_rotation = Eigen::Matrix3d::Identity();
  //create rotation matrix for cylinder center
  RPYToRotation(M_PI/2,0,0,plane_rotation);
  Eigen::Vector3d center_pt_wtf_plane;
  pcl::transformPoint(cylinder_params.center_pt,center_pt_wtf_plane,transform_cam2plane);
  Eigen::Affine3d center_wtf_plane_pose =  Eigen::Affine3d::Identity();
  center_wtf_plane_pose.translation() = center_pt_wtf_plane;
  center_wtf_plane_pose.rotate(plane_rotation);
  //transform to camera
  center_wtf_camera_pose = transform_cam2plane.inverse() * center_wtf_plane_pose;

  //compute grasping points
  std::vector<geometry_msgs::PoseStamped> grasping_poses(num_grasping_points_);
  calculateGraspingPoints(grasping_poses,point_cloud2_rgb_msg->header,*input_cloud,
                          cylinder_params,transform_cam2plane,center_wtf_camera_pose);



  //cylinder_center_coordinate.rotate(transform_cam2plane.rotation().inverse());

  ///Now everything should be correct, time to publish cylinder object information
  tf::Transform cylinder_center_tf;

  tf::transformEigenToTF(center_wtf_camera_pose,cylinder_center_tf);
  tf_broadcaster_.sendTransform(tf::StampedTransform(cylinder_center_tf,
                                                     ros::Time::now(),point_cloud2_rgb_msg->header.frame_id,
                                                     "cylinder_center"));


  //publish cylinder cloud
  if (debug_["publish_cylinder_object"] == true)
  {
    sensor_msgs::PointCloud2 cylinder_cloud_mgs;
    pcl::toROSMsg(*cylinder_cloud, cylinder_cloud_mgs);
    cylinder_cloud_mgs.header = point_cloud2_rgb_msg->header;
    cylinder_pub_.publish(cylinder_cloud_mgs);
  }






  //publish data


  ipa_rubbish_bin_detection::CylinderParams cylinder_params_msg;
  cylinder_params_msg.height = cylinder_params.height;
  cylinder_params_msg.radius = cylinder_params.radius;
  cylinder_params_msg.center_pt.push_back(cylinder_params.center_pt(0));
  cylinder_params_msg.center_pt.push_back(cylinder_params.center_pt(1));
  cylinder_params_msg.center_pt.push_back(cylinder_params.center_pt(2));
  cylinder_params_msg.direction_vec.push_back(cylinder_params.direction_vec(0));
  cylinder_params_msg.direction_vec.push_back(cylinder_params.direction_vec(1));
  cylinder_params_msg.direction_vec.push_back(cylinder_params.direction_vec(2));
  cylinder_params_msg.object_pose.pose.position.x = cylinder_params.center_pt(0);
  cylinder_params_msg.object_pose.pose.position.y = cylinder_params.center_pt(1);
  cylinder_params_msg.object_pose.pose.position.z = cylinder_params.center_pt(2);
  //update orientation
  cylinder_params_msg.object_pose.pose.orientation.x = axis_camera_cylinder.x() * sin(angle / 2);
  cylinder_params_msg.object_pose.pose.orientation.y = axis_camera_cylinder.y() * sin(angle / 2);
  cylinder_params_msg.object_pose.pose.orientation.z = axis_camera_cylinder.z() * sin(angle / 2);
  cylinder_params_msg.object_pose.pose.orientation.w = cos(angle / 2);
  cylinder_params_msg.object_pose.header.frame_id = point_cloud2_rgb_msg->header.frame_id;
  cylinder_params_msg.object_pose.header.stamp = ros::Time::now();
  cylinder_params_msg.grasping_poses = grasping_poses;

  //ROS_DEBUG_STREAM(cylinder_params_msg.object_pose);
  //publish cylinder parameters
  cylinder_params_pub_.publish(cylinder_params_msg);
  

  //publish visualization for cylinder
  if(debug_["show_cylinder_marker"] ==  true)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = point_cloud2_rgb_msg->header.frame_id;
    marker.header.stamp = ros::Time(0);
    marker.ns = "cylinder_shape";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = cylinder_params.center_pt(0);
    marker.pose.position.y = cylinder_params.center_pt(1);
    marker.pose.position.z = cylinder_params.center_pt(2);
    marker.pose.orientation.x = axis_camera_cylinder.x() * sin(angle / 2);
    marker.pose.orientation.y = axis_camera_cylinder.y() * sin(angle / 2);
    marker.pose.orientation.z = axis_camera_cylinder.z() * sin(angle / 2);
    marker.pose.orientation.w = cos(angle / 2);
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = cylinder_params.radius*2.0;
    marker.scale.y = cylinder_params.radius*2.0;
    marker.scale.z = cylinder_params.height;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub_.publish(marker);
  }


}

bool IpaWasteBasteDetection::cylinderDetection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cylinder_cloud,
                                               cv::Mat& color_image, const std_msgs::Header& header,
                                               pcl::ModelCoefficients::Ptr& plane_model, pcl::ModelCoefficients::Ptr& cylinder_model,
                                               Eigen::Affine3d& transform_cam2plane,CylinderParams& cylinder_params)
{

  bool found_plane = false;
  bool found_cylinder = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
      filtered_input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Build a passthrough filter to remove spurious NaNs
  pcl::PointIndices::Ptr pass_inliers(new pcl::PointIndices);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min_val_,z_max_val_);
  pass.filter (*filtered_input_cloud);

  //reduce points using voxel grid filter with 0.01 meters
  PointCloudProcessing::voxelGridFilter(filtered_input_cloud,filtered_input_cloud,0.01f);


  //plane segmentation
  pcl::PointIndices::Ptr inlier_indices (new pcl::PointIndices);
  found_plane = PointCloudProcessing::planeSegmentation(filtered_input_cloud,plane_model,inlier_indices,
                                                        floor_plane_inlier_distance_,min_plane_points_,floor_search_iterations_);


  if (!found_plane)
  {
    std::cout<<"no plane detected"<<std::endl;
    return false;
  }
  // extract segmented cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(filtered_input_cloud);
  extract.setIndices(inlier_indices);
  extract.setNegative(true);
  extract.filter(*segmented_cloud);

  if(segmented_cloud->points.size() == 0)
  {
    std::cout<<"plane cloud is empty"<<std::endl;
    return false;
  }
  //publish segmented cloud
  if (debug_["publish_floor_plane"] == true)
  {
    sensor_msgs::PointCloud2 segmented_cloud_mgs;
    pcl::toROSMsg(*segmented_cloud, segmented_cloud_mgs);
    segmented_cloud_mgs.header = header;
    floor_plane_pub_.publish(segmented_cloud_mgs);
  }



  //clustering
  if(!PointCloudProcessing::euclideanCluster(segmented_cloud,min_cluster_size_,100000,cluster_tolerance_,segmented_cloud))
  {
    std::cout<<"clustering failed"<<std::endl;
    return false;
  }
  //Remove outliers ??
  if(use_remove_outliers_)
  {
    PointCloudProcessing::statisticalOutliersRemovalFilter(segmented_cloud,40,0.02);
  }
  //extract normals
  pcl::PointCloud<pcl::Normal>::Ptr segmented_cloud_normals(new pcl::PointCloud<pcl::Normal>);
  PointCloudProcessing::computeNormals(segmented_cloud,segmented_cloud_normals);



  found_cylinder = PointCloudProcessing::extractCylinder(*segmented_cloud,*cylinder_cloud,cylinder_model,
                                                         segmented_cloud_normals,cylinder_distace_threshold_,cylinder_radius_,true);

  if(!found_cylinder)
  {

    //std::cout<<"PointCloudProcessing::extractCylinder failed"<<std::endl;
    return false;
  }

  ///Now the cylinder and plane are found (correct or not), time to get information
  /// TODO change camera normal in z negative direction (0,0,-1)
  /*
  Eigen::Vector3d plane_normal(plane_model->values[0],plane_model->values[1],plane_model->values[2]); //already normalized
  Eigen::Vector3d camera_normal(0,0,1);
  Eigen::Vector3d rot = camera_normal.cross(plane_normal).normalized();
  float theta = M_PI - std::acos(plane_normal.dot(camera_normal)); //angle between 2 planes

  transform_cam2plane.translation() << 0, 0, -plane_model->values[3]; // define a point on plane P(0,0,-d) => 0*a+0*b+
  transform_cam2plane.rotate (Eigen::AngleAxisd(theta, rot));
  */

  Eigen::Vector3d plane_normal(plane_model->values[0],plane_model->values[1],plane_model->values[2]); //already normalized
  Eigen::Vector3d camera_normal(0,0,-1);
  Eigen::Matrix3d cam2plane_rotation =
      Eigen::Quaterniond().setFromTwoVectors(plane_normal,camera_normal).toRotationMatrix();
  transform_cam2plane.translation() = Eigen::Vector3d(0, 0, -plane_model->values[3]); // define a point on plane P(0,0,-d) => 0*a+0*b+
  transform_cam2plane.rotate(cam2plane_rotation);


  //transform cylinder cloud from camera to plane coordinate
  pcl::transformPointCloud (*cylinder_cloud, *cylinder_cloud, transform_cam2plane);
  pcl::PointXYZRGB min_point,max_point;
  //get min,max points
  pcl::getMinMax3D(*cylinder_cloud,min_point,max_point);
  // Store the center point of cylinder relative to plane
  cylinder_params.center_pt(0) = static_cast<double>((min_point.x + max_point.x) / 2);
  cylinder_params.center_pt(1) = static_cast<double>((min_point.y + max_point.y) / 2);
  // cylinder_params.center_pt(2) = static_cast<double>((min_point.z + max_point.z) / 2)+floor_plane_inlier_distance_;
  cylinder_params.center_pt(2) = static_cast<double>(max_point.z / 2);

  //std::cout<<"camera: "<< cylinder_params.center_pt<<std::endl;
  //Transform center point from plane to camera coordinate
  pcl::transformPoint(cylinder_params.center_pt,cylinder_params.center_pt,transform_cam2plane.inverse());
  // Store the height of cylinder
  //cylinder_params.height = std::fabs(max_point.z - min_point.z) + floor_plane_inlier_distance_; //not sure??
  cylinder_params.height = std::fabs(max_point.z); //not sure??

  //compare with ground truth height, if the difference > 1.5 cm -> incorrect
  double height_difference = std::fabs(cylinder_params.height - ground_truth_cylinder_height_);
  if(check_ground_truth_cylinder_height_==true && height_difference >0.02)
  {
    std::cout<<"Check ground truth failed, difference: "<<height_difference<<std::endl<<"please measure the ground truth of cylnder and change value in yaml file"<<std::endl;
    return false;
  }
  cylinder_params.radius = static_cast<double>(cylinder_model->values[6]);
  /* Store direction vector of z-axis of cylinder. */
  cylinder_params.direction_vec(0) = static_cast<double>(cylinder_model->values[3]);
  cylinder_params.direction_vec(1) = static_cast<double>(cylinder_model->values[4]);
  cylinder_params.direction_vec(2) = static_cast<double>(cylinder_model->values[5]);




  if(debug_["show_cylinder_image"] == true)
  {
    pcl::PointCloud<pcl::PointXYZ> bounding_box_cloud;
    //create bounding box cloud and transform back to camera coordinate
    bounding_box_cloud.points.resize(8);
    bounding_box_cloud.points[0] = pcl::PointXYZ(min_point.x,max_point.y,max_point.z);
    bounding_box_cloud.points[1] = pcl::PointXYZ(max_point.x,max_point.y,max_point.z);
    bounding_box_cloud.points[2] = pcl::PointXYZ(max_point.x,min_point.y,max_point.z);
    bounding_box_cloud.points[3] = pcl::PointXYZ(min_point.x,min_point.y,max_point.z);
    bounding_box_cloud.points[4] = pcl::PointXYZ(min_point.x,max_point.y,min_point.z);
    bounding_box_cloud.points[5] = pcl::PointXYZ(max_point.x,max_point.y,min_point.z);
    bounding_box_cloud.points[6] = pcl::PointXYZ(max_point.x,min_point.y,min_point.z);
    bounding_box_cloud.points[7] = pcl::PointXYZ(min_point.x,min_point.y,min_point.z);
    pcl::transformPointCloud(bounding_box_cloud,bounding_box_cloud,transform_cam2plane.inverse());
    std::vector<cv::Point2f>  img_lines;
    for (int i=0;i<8;i++) {
      Eigen::Vector3d uv=cam_params_*Eigen::Vector3d(bounding_box_cloud.points[i].x,bounding_box_cloud.points[i].y,bounding_box_cloud.points[i].z);
      cv::Point2d pixel;
      pixel.x = uv(0)/uv(2) -0.5f;
      pixel.y = uv(1)/uv(2) -0.5f;
      img_lines.push_back(pixel);
    }
    cv::line(color_image,img_lines[0],img_lines[1],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[1],img_lines[2],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[2],img_lines[3],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[0],img_lines[4],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[3],img_lines[0],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[4],img_lines[5],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[5],img_lines[6],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[1],img_lines[5],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[6],img_lines[7],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[6],img_lines[2],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[7],img_lines[3],cv::Scalar(0,0,255));
    cv::line(color_image,img_lines[7],img_lines[4],cv::Scalar(0,0,255));
  }

  return true;

}



void IpaWasteBasteDetection::calculateGraspingPoints(std::vector<geometry_msgs::PoseStamped>& grasping_poses,
                                                     const std_msgs::Header& header,
                                                     pcl::PointCloud<pcl::PointXYZRGB>& original_cloud,
                                                     const CylinderParams& cylinder_params,
                                                     const Eigen::Affine3d& transform_cam2plane,
                                                     const Eigen::Affine3d& center_wtf_camera_pose)
{

  ////cut original point to reduce computation time
  pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud;
  //transform to plane
  pcl::transformPointCloud(original_cloud,segmented_cloud,transform_cam2plane);
  // Build a passthrough filter to  keep points in range (cylinder height - 10cm, cylinder heigt)
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(segmented_cloud.makeShared());
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(cylinder_params.height-0.1,cylinder_params.height);
  pass.filter (segmented_cloud);
  //transform back to camera
  pcl::transformPointCloud(segmented_cloud,segmented_cloud,transform_cam2plane.inverse());


  Eigen::Vector3d center_wtf_plane;
  pcl::transformPoint(cylinder_params.center_pt,center_wtf_plane,transform_cam2plane);
  //angle between 2 grasping point
  const double& angle = 360.0/num_grasping_points_;
  std::vector<Eigen::Vector3d> grasping_points(num_grasping_points_);


  for(int point=0,index = 2 ; point< num_grasping_points_;++point,++index)
  {
    const double& rad = (point)*angle*M_PI/180.0;

    //const double& rad = (index)*angle*M_PI/180.0;
    //offset x,y direction = 3cm, z= 10 cm to make radius of cylinder bigger
    const double& x_comp = (cylinder_params.radius+0.03)*cos(rad);
    const double& y_comp = (cylinder_params.radius+0.03)*sin(rad);
    //convert to affine3d
    Eigen::Affine3d grasping_pose_wtf_center = Eigen::Affine3d::Identity();
    Eigen::Affine3d grasping_pose_wtf_camera = Eigen::Affine3d::Identity();

    //Create quartenion from roll,pitch,yaw
    //RPYToPose(0,0,((index-2)*angle*M_PI)/180,grasping_poses[point].pose);
    Eigen::Matrix3d grasping_rot_wtf_center;
    RPYToRotation(0,0,((index-2)*angle*M_PI)/180,grasping_rot_wtf_center);
    grasping_pose_wtf_center.rotate(grasping_rot_wtf_center);
    grasping_pose_wtf_camera = center_wtf_camera_pose*grasping_pose_wtf_center;

    tf::poseEigenToMsg(grasping_pose_wtf_camera,grasping_poses[point].pose);

    Eigen::Vector3d grasping_point_wtf_center(x_comp,y_comp,cylinder_params.height/2+0.1);
    //Eigen::Vector3d grasping_point_wtf_center(x_comp,y_comp,0);
    Eigen::Vector3d grasping_point_wtf_plane = center_wtf_plane + grasping_point_wtf_center;
    //transform to camera
    pcl::transformPoint(grasping_point_wtf_plane,grasping_points[point],transform_cam2plane.inverse());

  }
  std::vector<double> min_distance(grasping_points.size(),99999);
  std::vector<double> diffences(grasping_points.size());
  std::vector<int> indices(grasping_points.size());

  for (std::size_t i=0;i<segmented_cloud.points.size();i++) {
    Eigen::Vector3d p = Eigen::Vector3d(segmented_cloud.points[i].x,
                                        segmented_cloud.points[i].y,segmented_cloud.points[i].z);
    for (int point = 0; point<num_grasping_points_;point++) {
      diffences[point] = (p - grasping_points[point]).norm();
      if(min_distance[point]>diffences[point])
      {
        indices[point] = i;
        min_distance[point] = diffences[point];
      }
    }
  }

  for(int i=0,index = 2;i<num_grasping_points_;i++,index ++)
  {
    grasping_poses[i].header = header;
    grasping_poses[i].header.stamp = ros::Time::now();


    std::ostringstream ss;
    ss<<i;

    grasping_poses[i].pose.position.x = segmented_cloud.points[indices[i]].x;
    grasping_poses[i].pose.position.y = segmented_cloud.points[indices[i]].y;
    grasping_poses[i].pose.position.z = segmented_cloud.points[indices[i]].z;


    tf::Transform transform;
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(grasping_poses[i].pose, pose);
    tf::transformEigenToTF(pose,transform);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform,
                                                       ros::Time::now(),header.frame_id,
                                                       "pt"+ss.str()));
  }

}



int main(int argc, char** argv) {

  ros::init(argc, argv, "wastebasket_detection_node");
  ros::NodeHandle node;
  IpaWasteBasteDetection::Ptr waste_baste_detecction = IpaWasteBasteDetection::Ptr(new IpaWasteBasteDetection(node));
  ros::spin();

  return 0;
}

