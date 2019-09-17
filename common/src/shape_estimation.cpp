#include "ipa_rubbish_bin_detection/shape_estimation.hpp"


void PointCloudProcessing::computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}
void PointCloudProcessing::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}


void PointCloudProcessing::voxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelgrid_filter;
  voxelgrid_filter.setInputCloud(input_cloud);
  voxelgrid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxelgrid_filter.filter(*output_cloud);
  //printf("filtered cloud using voxelgrid filter using leaf size: %f \n",leaf_size);
}
void PointCloudProcessing::passThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
                                             const std::string field_name,const float min_value,const float max_value)
{
  //printf("Number of points before passthrough : %ld points in %s direction\n",input_cloud->points.size(),field_name.c_str());
  pcl::PointCloud<pcl::PointXYZRGB> tmp;
  pcl::PassThrough<pcl::PointXYZRGB> bbox_filter;
  bbox_filter.setFilterFieldName(field_name);
  bbox_filter.setFilterLimits(min_value, max_value);
  bbox_filter.setInputCloud(input_cloud);
  bbox_filter.filter(*output_cloud);
  // printf("Number of points after passthrough : %ld \n",output_cloud->points.size());
}

void PointCloudProcessing::cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
                                   float min_x,float min_y,float min_z,
                                   float max_x,float max_y,float max_z)
{
  //printf("Number of points before cropBox : %ld points\n",input_cloud->points.size());
  pcl::CropBox<pcl::PointXYZRGB> boxFilter(true);

  boxFilter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  boxFilter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  boxFilter.setInputCloud(input_cloud);
  boxFilter.filter(*output_cloud);
  //printf("Number of points after passthrough : %ld \n",output_cloud->points.size());
}

bool PointCloudProcessing::planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                             pcl::ModelCoefficients::Ptr& plane_model,
                                             pcl::PointIndices::Ptr& inlier_indices,const double distance_threshold, const int min_plane_points,
                                             const int floor_search_iterations,
                                             bool debug)
{
  bool found_plane = false;
  if(debug == true)
  {

    cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
    int index = 0;
    for (int v = 0; v < (int) input_cloud->height; v++)
    {
      for (int u = 0; u < (int) input_cloud->width; u++, index++)
      {
        pcl::PointXYZRGB& point = (*input_cloud)[index];
        color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
      }
    }
    //std::cout << input_cloud->height << "x" << input_cloud->width << std::endl;

    //display original image
    cv::imshow("original color image", color_image);
    cvMoveWindow("original color image", 0, 0);
    cv::waitKey(1);
    //cvMoveWindow("color image", 0, 520);
  }
  // plane coefficients
  for (int trial = 0; (trial < floor_search_iterations && input_cloud->points.size() > min_plane_points); trial++)
  {
    inlier_indices->indices.clear();
    // segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(input_cloud);
    seg.segment(*inlier_indices, *plane_model);
    // keep plane_normal upright
    if (plane_model->values[2] < 0.)
    {
      plane_model->values[0] *= -1;
      plane_model->values[1] *= -1;
      plane_model->values[2] *= -1;
      plane_model->values[3] *= -1;
    }
    // verify that plane is a valid ground plane
    if ((int) inlier_indices->indices.size() > min_plane_points)
    {
      found_plane = true;
      break;
    }
  }
  return found_plane;
}

bool PointCloudProcessing::extractSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sphere,
                                         pcl::ModelCoefficients::Ptr& coefficients,
                                         pcl::PointIndices::Ptr& inlier_indices,
                                         const int max_iterations, const double distance_threshold,
                                         const double min_radius_limit, const double max_radius_limit,
                                         bool extract_sphere)
{

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);

  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);

  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(input_cloud);
  seg.setRadiusLimits(min_radius_limit, max_radius_limit);

  seg.segment(*inlier_indices, *coefficients);

  // extract sphere
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  extract.setInputCloud(input_cloud);
  extract.setIndices(inlier_indices);
  extract.setNegative(extract_sphere);
  extract.filter(*cloud_sphere);
  std::cout<<"PointCloudProcessing::sphereSegmentation:"<<std::endl;
  std::cout << "sphere coefficients: " << *coefficients << std::endl;
  std::cout << "Number of inliers: " << inlier_indices->indices.size() << std::endl;
}
bool PointCloudProcessing::extractCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_circle,
                                         pcl::PointIndices::Ptr& inlier_indices, const double distance_threshold,
                                         bool extract_circle)
{

  // plane coefficients
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);

  seg.setInputCloud(input_cloud);
  seg.segment(*inlier_indices, *coefficients);

  // extract circle
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  extract.setInputCloud(input_cloud);
  extract.setIndices(inlier_indices);
  extract.setNegative(extract_circle);
  extract.filter(*cloud_circle);
  std::cout<<"PointCloudProcessing::circleSegmentation:"<<std::endl;
  std::cout << "circle coefficients: " << *coefficients << std::endl;
  std::cout << "Number of inliers: " << inlier_indices->indices.size() << std::endl;
}
bool PointCloudProcessing::extractCylinder( pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
                                            pcl::PointCloud<pcl::PointXYZRGB>& cloud_out,
                                            pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                                            pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                                            const double distance_threshold,const double radius,
                                            bool extract_cylinder)
{
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(10000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(distance_threshold);
  // min max values of radius in meters to consider
  segmentor.setRadiusLimits(0, radius);
  segmentor.setInputCloud(cloud_in.makeShared());
  segmentor.setInputNormals(cloud_normals->makeShared());

  // Obtain the cylinder inliers and coefficients
  segmentor.segment(*inliers_cylinder, *coefficients_cylinder);
  // Extract the cylinder inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_in.makeShared());
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(cloud_out);
  if (cloud_out.points.empty ()) {
    // std::cerr << "Can't find the cylindrical component." << std::endl;
    return false;
  }
  //std::cerr << "PointCloud representing the cylindrical component: " << cloud_out.points.size () << " data points." << std::endl;
  return true;
}
bool PointCloudProcessing::extractCylinder( pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                     pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                     pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                      pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                     const double distance_threshold,const double radius,
                     bool extract_cylinder)
{
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(10000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(distance_threshold);
  // min max values of radius in meters to consider
  segmentor.setRadiusLimits(0, radius);
  segmentor.setInputCloud(cloud_in.makeShared());
  segmentor.setInputNormals(cloud_normals->makeShared());

  // Obtain the cylinder inliers and coefficients
  segmentor.segment(*inliers_cylinder, *coefficients_cylinder);
  // Extract the cylinder inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_in.makeShared());
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  extract.filter(cloud_out);
  if (cloud_out.points.empty ()) {
    // std::cerr << "Can't find the cylindrical component." << std::endl;
    return false;
  }
  std::cerr << "PointCloud representing the cylindrical component: " << cloud_out.points.size () << " data points." << std::endl;
  return true;
}




void PointCloudProcessing::statisticalOutliersRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const int number_of_neighbours, const double threshold)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sorFilter;
  sorFilter.setInputCloud(cloud);
  sorFilter.setMeanK(number_of_neighbours);
  sorFilter.setStddevMulThresh(threshold);
  sorFilter.filter(*cloud);
  //std::cout << "removed outliers using statistical outlier removal filter with a std deviation of: " << threshold<< std::endl;
}

bool PointCloudProcessing::euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min_size, int max_size, double tolerance,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output)
{

  if(cloud->points.size()==0)
  {
    std::cout<<"input cloud is empty, can not create clustering"<<std::endl;
    return false;
  }
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > output_clouds;
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  if(cluster_indices.empty())
  {
    return false;
  }
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr max_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  //  for(std::vector<int>::const_iterator pit = cluster_indices[0].indices.begin();pit !=cluster_indices[0].indices.end();++pit)
  //    output->points.push_back (cloud->points[*pit]); //*

  //  output->width = output->points.size();
  //  output->height = 1;
  //  output->is_dense = false;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = false;
    output_clouds.push_back(cloud_cluster);
  }
  *output = *output_clouds[0];
  return true;
}

