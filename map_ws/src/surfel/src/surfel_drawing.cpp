#include <iostream>
#include <pcl/io/pcd_io.h>
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_cluster.pcd", *cloud_filtered_p) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the file! \n");
    return (-1);
  }
  
  
  
  std::vector<pcl::ModelCoefficients> cone_coeffs;
  
  std::ifstream infile("surfels.txt");
  float values[7];
  while (infile >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >> values[5] >> values[6])
  {
    pcl::ModelCoefficients cone_coeff;
    cone_coeff.values.resize (7);    // We need 7 values
    cone_coeff.values[0] = values[0];
    cone_coeff.values[1] = values[1];
    cone_coeff.values[2] = values[2];
    cone_coeff.values[3] = values[3];
    cone_coeff.values[4] = values[4];
    cone_coeff.values[5] = values[5];
    cone_coeff.values[6] = values[6]; // degrees
    
    cone_coeffs.push_back(cone_coeff);
  }
  infile.close();
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  for(int i=0; i<cone_coeffs.size(); i++){
    std::string name="cone";
    name+=std::to_string(i);
    cout<<"Nazywa sie: "<<name<<endl;
    viewer->addCone(cone_coeffs[i], name);
  }
  
  viewer->setRepresentationToSurfaceForAllActors(); 
  for(int i=0; i<cone_coeffs.size(); i++){
    std::string name="cone";
    name+=std::to_string(i);
    float r,g,b, sum;
    sum = cone_coeffs[i].values[3] + cone_coeffs[i].values[4] + cone_coeffs[i].values[5];
    r = cone_coeffs[i].values[3]/sum;
    g = cone_coeffs[i].values[4]/sum;
    b = cone_coeffs[i].values[5]/sum;
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name);
  }
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered_p, "cloud");
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


  return (0);
}
