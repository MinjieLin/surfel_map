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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smaller (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr podloga (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector<int>);
  
  
  pcl::PCDWriter writer;
  
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
  
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (cloud_filtered);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_smaller);
 

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;


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
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  
  std::vector<pcl::ModelCoefficients> cone_coeffs;
 
 
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr current_cluster_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr current_cluster_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfel_points (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr konce (new pcl::PointCloud<pcl::PointXYZRGB>);
 
 
  int j=0;
  for(int i=0; i<clusters.size(); i++){
    
    pcl::IndicesPtr clusterindices (new std::vector<int>(clusters[i].indices));
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract0;
    extract0.setInputCloud (colored_cloud);
    extract0.setIndices (clusterindices);
    extract0.setNegative (false);
    extract0.filter (*current_cluster);
    

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
  
    seg.setInputCloud (current_cluster);
    seg.segment (*inliers, *coefficients);
  
  
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                    << coefficients->values[1] << " "
  //                                    << coefficients->values[2] << " " 
  //                                    << coefficients->values[3] << std::endl;
  //                                    
  //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  //std::cerr << "Size of first cluster: " << first_cluster->size () << std::endl;
  
  Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  
  
     // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (current_cluster);
  proj.setModelCoefficients (coefficients);
  proj.filter (*current_cluster_proj);
  
  while(current_cluster_proj->size()>20){
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*current_cluster_proj, centroid);
  
    //cout<< "Centroid: "<< endl << centroid[0] << endl << centroid[1] << endl << centroid[2] << endl; 
  
  
    pcl::PointXYZRGB end_point_1;
    pcl::PointXYZRGB end_point_2;
  
    pcl::getMinMax3D (*current_cluster_proj, end_point_1, end_point_2);
  
    //cout << "End point 1: " << end_point_1 << endl;
    //cout << "End point 2: " << end_point_2 << endl;
    
    int v1 = rand() % current_cluster_proj->size();
    pcl::PointXYZRGB surf_center;
    surf_center=current_cluster_proj->points[v1];
    
  
    konce->push_back(end_point_1);
    konce->push_back(end_point_2);
  
    float distance = sqrtf(pow(end_point_1.x-end_point_2.x,2)+pow(end_point_1.y-end_point_2.y,2)+pow(end_point_1.z-end_point_2.z,2));
    float radius=distance/4;
  
    int old_percentage=0;
    int old_k = 0;
    float last_gain = 0; // stosunek ilosci punktow do powierzchni
    pcl::IndicesPtr probably_used_points (new std::vector<int>);
    for(float k=radius; k<distance/2; k+=0.05){
      pcl::IndicesPtr used_points (new std::vector<int>);
      std::cout << "Badany promien: " << k << endl;
      float gain=0;
      int percentage=0;
      for(int i=0; i<current_cluster_proj->size(); i++){
        pcl::PointXYZRGB poi = current_cluster_proj->points[i];
        if((pow(poi.x-surf_center.x,2)+pow(poi.y-surf_center.y,2)+pow(poi.z-surf_center.z,2))<pow(k,2)){
          percentage++;
          used_points->push_back(i);
        }
        gain = percentage/(k*k);
      }
      percentage=percentage*100;
      percentage=percentage/current_cluster_proj->size();
      //if(percentage>old_percentage && percentage<90){
      if(gain>last_gain){
        last_gain = gain;
        old_percentage=percentage;
        old_k=k;
        radius=k;
        probably_used_points=used_points;
        
      }
      else break;
    }
    std::cout << "Wybrany promien: " << radius << ", rozpietosc chmury: " << distance << endl;
  
  
    //cout << "Distance: " << distance << endl;
    //cout << "Surfel radius: " << radius << endl;
  

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud (current_cluster_proj);
    extract_indices.setIndices (probably_used_points);
    extract_indices.setNegative (true);
    extract_indices.filter (*current_cluster_proj);
  
    int deg=82;
    float rad = 1.43116999; //deg in radians
  
    // Draw a surfel //
    pcl::ModelCoefficients cone_coeff;
    cone_coeff.values.resize (7);    // We need 7 values
    cone_coeff.values[0] = surf_center.x;
    cone_coeff.values[1] = surf_center.y;
    cone_coeff.values[2] = surf_center.z;
    cone_coeff.values[3] = normal[0]*(radius/tan(rad));
    cone_coeff.values[4] = normal[1]*(radius/tan(rad));
    cone_coeff.values[5] = normal[2]*(radius/tan(rad));
    cone_coeff.values[6] = deg; // degrees
    
    cone_coeffs.push_back(cone_coeff);
    }
  
  
  
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color2(current_cluster_proj, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> kolor_konce(konce, 200, 5, 5);
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_filtered);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, rgb, "all");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "all");

  
  
  viewer->addPointCloud<pcl::PointXYZRGB> (current_cluster_proj, single_color2, "projected");
  viewer->addPointCloud<pcl::PointXYZRGB> (konce, kolor_konce, "konce");
  for(int i=0; i<cone_coeffs.size(); i++){
    std::string name="cone";
    name+=std::to_string(i);
    cout<<"Nazywa sie: "<<name<<endl;
    viewer->addCone(cone_coeffs[i], name);
  }
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "projected");
  
  //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud_filtered_p, normals_p, 20, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
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
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  //writer.write<pcl::PointXYZRGB> ("first_cluster2.pcd", *first_cluster, false);

  return (0);
}
