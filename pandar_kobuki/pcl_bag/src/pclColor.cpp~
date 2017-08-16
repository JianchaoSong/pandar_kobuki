/// the goal of this cpp is make the pcd colorful
/// author : jc
/// time   : 2017.07.29 

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pcl/filters/approximate_voxel_grid.h>


int main(int argc,char ** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_show_low(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_show_mid (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_show_up(new pcl::PointCloud<pcl::PointXYZ>);
  char * dir;
  dir=argv[1];//"/home/jc/hesai_ws/pandar_workspace/1501143985845585.pcd"
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(dir,*cloud_in)==-1)//打开点云文件
  {
	PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //文件名要写对
	std::cout<<"usage:: rosrun pcl_bag pclColor /home/jc/hesai_ws/pandar_workspace/1501143985845585.pcd"<<std::endl;
	return -1;
	
  }
  // the height determine the color of a point in cloud_in and set it as color
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in,indices);
  pcl::PointXYZ point;
  double height=0;
  std::cout<<"point size is :"<<cloud_in->points.size()<<std::endl;
  for(int i=0;i<cloud_in->points.size();i++)
  {
    point.x=cloud_in->points[i].x;
    point.y=cloud_in->points[i].y;
    point.z=cloud_in->points[i].z;
    // the result of blam is z 
/*
    if(point.z<=-0.65)
    {
      cloud_show_low->push_back(point);
    }
    if(point.z<1.7&& point.z>-0.65)
    {
      cloud_show_mid->push_back(point);
    }
    if(point.z>=1.7)
    {
      cloud_show_up->push_back(point);
    }
    if(point.y>height)
      height=point.z;
*/
    if(point.y<=-0.65)
    {
      cloud_show_low->push_back(point);
    }
    if(point.y<1.4&& point.y>-0.65)
    {
      cloud_show_mid->push_back(point);
    }
    if(point.y>=1.4)
    {
      cloud_show_up->push_back(point);
    }
    if(point.y>height)
      height=point.y;
  }
  std::cout<<"max height" <<height<<std::endl;
 // pcl::visualization::PointCloudColorHandler<pcl::PointXYZ> target_color();
  //显示结果图
  /*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);  //设置背景
  viewer->addPolygonMesh(triangles,"my");  //设置显示的网格
  viewer->addCoordinateSystem(1.0);  //设置坐标系
  viewer->initCameraParameters();
  while(!viewer->wasStopped())
  {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
  */
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);  //设置背景颜色为黑色
  // 对目标点云着色可视化 (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (cloud_show_low, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (cloud_show_low, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // 对转换后的源点云着色 (green)可视化.
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (cloud_show_mid, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (cloud_show_mid, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");
  // 对转换后的源点云着色 (green)可视化.
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color1 (cloud_show_up, 0, 0, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (cloud_show_up, output_color1, "output1 cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // 启动可视化
  viewer_final->addCoordinateSystem (1.0);  //显示XYZ指示轴
  viewer_final->initCameraParameters ();   //初始化摄像头参数

  // 等待直到可视化窗口关闭
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

}
