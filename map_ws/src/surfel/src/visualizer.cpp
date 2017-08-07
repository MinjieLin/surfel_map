#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>

int main (int argc, char** argv)
{
  
   if(argc < 2){
    cout << "Name the cloud you want to view!\n";
    return 0;
  }
  std::string name;
  name = argv[1];
  cout << name;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the file! \n");
    return (-1);
  }
  std::cout << "Loaded cloud is "<< cloud->width << " x " << cloud->height<< "."<< std::endl;
            
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  
    while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  
  return(0);
}
  
  
