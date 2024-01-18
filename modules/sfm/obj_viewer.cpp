#include <iostream>
#include <thread>
#include <execution>
#include <future>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

struct S1
{
  S1()
  {
    std::cout << "S1 constructor()" << std::endl;
  }
  S1(const S1& other)
   : value(other.value)
  {
    std::cout << "S1 copy constructor()" << std::endl;
  }
  S1(S1&& other)
   : value(std::move(other.value))
  {
    std::cout << "S1 move constructor()" << std::endl;
  }

  S1& operator=(const S1& other)
  {
    this->value = other.value;
    std::cout << "S1 copy assignment()" << std::endl;

    return *this;
  }

  int value = 0;
};
struct Container
{
  Container()
  {
    std::cout << "Container constructor()" << std::endl;
  }
  Container(const Container& other)
   : value(other.value)
  {
    std::cout << "Container copy constructor()" << std::endl;
  }
  Container(Container&& other)
   : value(std::move(other.value))
  {
    std::cout << "Container move constructor()" << std::endl;
  }

  Container& operator=(const Container& other)
  {
    this->value = other.value;
    std::cout << "Container copy assignment()" << std::endl;

    return *this;
  }

  S1 value;
};

void doo1()
{
  std::cout << "doo1\n";
}

int main (int argc, char** argv)
{
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // if (pcl::io::loadOBJFile<pcl::PointXYZRGB> (
  //     "/home/blackdyce/Projects/AI/Datasets/lulu/base.obj", 
  //     *cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read file \n");
  //   return (-1);
  // }
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height
  //           << " data points from FILE with the following fields: "
  //           << std::endl;
            

  // pcl::visualization::CloudViewer viewer ("Viewer");
  // viewer.showCloud (cloud);

  // while (!viewer.wasStopped ())
  // {
  // }

  // S1 s1;
  // S1 s2 = s1;
  // S1 s3 {s1};
  // s3 = s1;

  Container c1;
  Container c2 = c1;
  // c2 = c1;

  std::cout << "endl\n";


  return (0);
}