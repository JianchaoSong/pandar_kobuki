#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.  

//字符串连接  
char* str_contact(const char* str1,const char* str2){
    char* result;  
    result=(char*)malloc(strlen(str1)+strlen(str2)+1);//str1的长度+str2的长度+\0;  
    if(!result){//如果内存动态分配失败  
        printf("字符串连接时，内存动态分配失败\n");  
        exit(1);  
    }  
    strcat(result,str1);  
    strcat(result,str2);//字符串拼接  
    return result;  
} 


int 
main (int argc, char **argv)  
{  
  ros::init (argc, argv, "pclToBag");  
  ros::NodeHandle nh;  
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pandar_node/pandar_points", 1);  
  pcl::PointCloud<pcl::PointXYZ> cloud;  
  //sensor_msgs::PointCloud2 output;  
  
 // output.header.frame_id = "pandar";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer  
  ros::Rate loop_rate(10);     
  int i=1;
  sensor_msgs::PointCloud2 output;  
   output.header.frame_id = "/pandar";
  while (ros::ok())  
  {  
    if(i<1263)
    {
      
      //string s;
       string s;
       s=argv[1];//"/home/jc/pcd_0727/pcl_";
       s+="/pcl_";
       char numstr[5];
      sprintf(numstr,"%d",i);
      s+=numstr;
      s+="_measure.pcd";
     
      pcl::io::loadPCDFile (s.c_str(), cloud);  
      ROS_INFO("%s Get a cloud ---- size:%d",s.c_str(),int (cloud.size()));
      //Convert the cloud to ROS message  
      pcl::toROSMsg(cloud, output);  
      //在最后要添加这些信息，因为上面的toRosMsg，会把cloud中的赋值给output
      //其中不包括frame_id,stamp,seq
      output.header.frame_id = "/pandar";
      output.header.stamp=ros::Time::now();
      output.header.seq=i;
      pcl_pub.publish(output);           
    }
    ros::spinOnce();  
    loop_rate.sleep();  
    i++;
    if(i==1263)
      return 0;
  }  
  return 0;  
}  
