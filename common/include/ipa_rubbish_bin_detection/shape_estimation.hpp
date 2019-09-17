#ifndef SHAPE_ESTIMATION_HPP
#define SHAPE_ESTIMATION_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
//#include <pcl/features/rsd.h>
//#include <pcl/features/grsd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
namespace PointCloudProcessing{

void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);
void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);

void voxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud, float leaf_size);

void passThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
                       const std::string field_name,const float min_value,const float max_value);

void cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
             float min_x,float min_y,float min_z,
             float max_x,float max_y,float max_z);

bool planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                        pcl::ModelCoefficients::Ptr& plane_model,
                        pcl::PointIndices::Ptr& inlier_indices,const double distance_threshold, const int min_plane_points,const int floor_search_iterations,
                        bool debug = false);
bool extractSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sphere,
                        pcl::ModelCoefficients::Ptr& coefficients,
                        pcl::PointIndices::Ptr& inlier_indices,
                        const int max_iterations, const double distance_threshold,
                        const double min_radius_limit, const double max_radius_limit,
                        bool extract_sphere = false);
bool extractCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_circle,
                        pcl::PointIndices::Ptr& inlier_indices, const double distance_threshold,
                        bool extract_circle = false);
bool extractCylinder( pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
                     pcl::PointCloud<pcl::PointXYZRGB>& cloud_out,
                     pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                      pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                     const double distance_threshold,const double radius,
                     bool extract_cylinder);
bool extractCylinder( pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                     pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                     pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                      pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                     const double distance_threshold,const double radius,
                     bool extract_cylinder);
void statisticalOutliersRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const int number_of_neighbours, const double threshold);
bool euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min_size, int max_size, double tolerance,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output);

};


#endif  // SHAPE_ESTIMATION_HPP
