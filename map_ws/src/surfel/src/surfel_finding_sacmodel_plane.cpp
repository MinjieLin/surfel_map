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

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::IndicesPtr indices (new std::vector<int>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_filtered.pcd", *cloud_filtered) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the file! \n");
    return (-1);
  }
  std::cout << "Loaded cloud is "
            << cloud_filtered->width << " x " << cloud_filtered->height
            << "."
            << std::endl;
            
  for (int i = 0; i < cloud_filtered->width * cloud_filtered->height; i+=30) indices->push_back(i);
  
  
  //////////////////////////////////// downsampling the cloud //////////////////////////// 
  
  //plaszczyzny
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::PointCloud <pcl::Normal>::Ptr normals_p (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);
  
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
  
  /////////////////////////////// finding clusters /////////////////////////////////////
  
  
  
  
  
  
  
  ///////////////////////// euclidean cluster extraction ///////////////////////////////////
    // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree2->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree2);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
    int j = 0;
    std::vector<pcl::ModelCoefficients> cone_coeffs;
    
    
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr current_cluster_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr current_cluster_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfel_points (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr konce (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      current_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    current_cluster->width = current_cluster->points.size ();
    current_cluster->height = 1;
    current_cluster->is_dense = true;
    
    

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
  
  
  if (inliers->indices.size () == 0){
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    
    
    
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (current_cluster);
    proj.setModelCoefficients (coefficients);
    proj.filter (*current_cluster_proj);
  
    while(current_cluster_proj->size()>20){
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*current_cluster_proj, centroid);
  
  
      pcl::PointXYZRGB end_point_1;
      pcl::PointXYZRGB end_point_2;
  
      pcl::getMinMax3D (*current_cluster_proj, end_point_1, end_point_2);
    
      int v1 = rand() % current_cluster_proj->size();
      pcl::PointXYZRGB surf_center;
      surf_center=current_cluster_proj->points[v1];
    
  
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


  
  /////////////////////////end of euclidean cluster extraction ///////////////////////////////////
  
 
 ///////////////////////// operations on clusters ///////////////////////////////////
  int jj=0;
  ofstream surfel_list;
  surfel_list.open ("surfels.txt");
  for(int i=0; i<cone_coeffs.size(); i++){
    for(int j=0; j<=6; j++){
      surfel_list << cone_coeffs[i].values[j] << " ";
    }
    surfel_list << endl;
    
  
  }
  
  surfel_list.close();
  return (0);
  
  
}

