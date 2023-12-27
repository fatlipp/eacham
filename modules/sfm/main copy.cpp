#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (
      "/home/blackdyce/Projects/AI/NERF_PROJECTS/my_nerf/Projects/nerf_pl/test.ply", 
      *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from FILE with the following fields: "
            << std::endl;
            

  pcl::visualization::CloudViewer viewer ("Viewer");
  viewer.showCloud (cloud);

  while (!viewer.wasStopped ())
  {
  }

  return (0);
}