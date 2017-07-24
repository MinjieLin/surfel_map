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
#include <pcl/PolygonMesh.h>
#include <pcl/common/geometry.h>


int main (int argc, char** argv)
{
  if(argc < 2){
    cout << "Name the cloud you want to clean!\n";
    return 0;
  }
  std::string name;
  name = argv[1];
  cout << name;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PCDWriter writer;
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the file! \n");
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


  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  
  writer.write<pcl::PointXYZRGB> ("cloud_filtered.pcd", *cloud_filtered, false);

  
  return (0);
}
