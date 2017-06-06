#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>

 #include <pcl/point_cloud.h>
 #include <pcl/point_traits.h>
 #include <pcl/PointIndices.h>
 #include <pcl/cloud_iterator.h>


int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smaller (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr podloga (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector<int>);
  //cl::CentroidPoint<pcl::PointXYZRGB>::Ptr centroid (new pcl::CentroidPoint<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud3-3-2-2017.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded cloud is "
            << cloud->width << " x " << cloud->height
            << "."
            << std::endl;
            
            
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
         
  for (int i = 0; i < cloud_filtered->width * cloud_filtered->height; i+=30) indices->push_back(i);
  //for (int i = 0; i < 100000; i+=1) indices->push_back(i);
  
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_filtered);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_smaller);
 

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;

  // Create the normal estimation class, and pass the input dataset to it
  //pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  //ne.setInputCloud (cloud_filtered);
  //ne.setIndices(indices);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  //ne.setSearchMethod (tree);

  // Output datasets
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  

  // Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (0.1);

  // Compute the features
  //ne.compute (*cloud_normals);
  
  //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  
  //pcl::concatenateFields(*cloud_filtered, *cloud_normals, *cloud_with_normals);
  
  
  //one plane
  /*pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  
  // Extract the inliers
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  
  extract.setNegative (true);
  extract.filter (*cloud_filtered_p);*/
  
  
  //bez podlogi i sufitu
  /*pcl::PassThrough<pcl::PointXYZRGB> pass2;
  pass2.setInputCloud (cloud_filtered);
  pass2.setFilterFieldName ("z");
  pass2.setFilterLimits (0.1, 2.4);
  //pass.setFilterLimitsNegative (true);
  pass2.filter (*podloga);*/


  //plaszczyzny
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::PointCloud <pcl::Normal>::Ptr normals_p (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  //pcl::IndicesPtr indices2 (new std::vector <int>);
  //pcl::PassThrough<pcl::PointXYZRGB> pass;
  //pass.setInputCloud (cloud_smaller);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices2);
  
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (indices);
  extract.setNegative (false);
  extract.filter (*cloud_filtered_p);
  
  pcl::ExtractIndices<pcl::Normal> extract_norm;
  extract_norm.setInputCloud (normals);
  extract_norm.setIndices (indices);
  extract_norm.setNegative (false);
  extract_norm.filter (*normals_p);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (10);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud_filtered_p);
  //reg.setIndices (indices);
  reg.setInputNormals (normals_p);
  reg.setSmoothnessThreshold (8.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[3].indices.size () << " points." << endl;
 
   // build the condition
  int rMin = 253;
  int gMax = 1;
  int bMax = 1;
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMax)));

  // build the filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
  condrem.setInputCloud (colored_cloud);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*colored_cloud_p); 
  
  // view only the first cluster
  pcl::IndicesPtr clusterindices (new std::vector<int>(clusters[3].indices));
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr first_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr first_cluster_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr first_cluster_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::ExtractIndices<pcl::PointXYZRGB> extract0;
  extract0.setInputCloud (colored_cloud);
  extract0.setIndices (clusterindices);
  extract0.setNegative (false);
  extract0.filter (*first_cluster);
  
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  
  
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (first_cluster);
  seg.segment (*inliers, *coefficients);
  
  
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
                                      
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  std::cerr << "Size of first cluster: " << first_cluster->size () << std::endl;
  
   // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (first_cluster);
  proj.setModelCoefficients (coefficients);
  proj.filter (*first_cluster_proj);
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*first_cluster_proj, centroid);
  
  cout<< "Centroid: "<< endl <<centroid[0] << endl << centroid[1] << endl << centroid[2] << endl; 
  
  
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize (7);    // We need 7 values
  cylinder_coeff.values[0] = centroid[0];
  cylinder_coeff.values[1] = centroid[1];
  cylinder_coeff.values[2] = centroid[2];
  cylinder_coeff.values[3] = 1;
  cylinder_coeff.values[4] = 0;
  cylinder_coeff.values[5] = 0;
  cylinder_coeff.values[6] = 1;
  
  
  pcl::ModelCoefficients cylinder2_coeff;
  cylinder2_coeff.values.resize (7);    // We need 7 values
  cylinder2_coeff.values[0] = centroid[0];
  cylinder2_coeff.values[1] = centroid[1]+2;
  cylinder2_coeff.values[2] = centroid[2];
  cylinder2_coeff.values[3] = 2;
  cylinder2_coeff.values[4] = 0;
  cylinder2_coeff.values[5] = 0;
  cylinder2_coeff.values[6] = 1;

  
  
  
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud (first_cluster);
  extract_indices.setIndices (inliers);
  extract_indices.setNegative (false);
  extract_indices.filter (*first_cluster_p);
  
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(first_cluster);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(first_cluster_p, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(first_cluster_proj, 255, 255, 255);
  
  //viewer->addPointCloud<pcl::PointXYZRGB> (first_cluster, rgb, "all");
  //viewer->addPointCloud<pcl::PointXYZRGB> (first_cluster_p, single_color, "inliers");
  viewer->addPointCloud<pcl::PointXYZRGB> (first_cluster_proj, single_color2, "projected");
  
  //viewer->addPlane(*coefficients, -10, 0, 0);
  
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inliers");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "projected");
  
  //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_filtered_p, normals_p, 20, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  viewer->addCylinder (cylinder_coeff, "cylinder");
  viewer->addCylinder (cylinder2_coeff, "cylinder2");
  
  viewer->setRepresentationToSurfaceForAllActors(); 
  
  
  //pcl::visualization::CloudViewer viewer2 ("Cluster viewer");
  //viewer2.showCloud(first_cluster);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  
  //writer.write<pcl::PointXYZRGB> ("colored_cloud.pcd", *colored_cloud_p, false);
  
  
  

  return (0);
}
