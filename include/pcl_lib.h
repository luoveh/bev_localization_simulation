#ifndef _PCL_LIB_H_
#define _PCL_LIB_H_

#include <pcl/io/pcd_io.h>          
#include <pcl/point_types.h>        
#define BOOST_TYPEOF_EMULATION  
#include <pcl/registration/icp.h>   
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

#endif //#ifndef _PCL_LIB_H_