//author: jc S

//time  : 2017.07.29

//working for remap the pandar_points to 2d laserScan type points

#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

/* 
 * @@ for convert the pointcloud2 to laserScan
 * get the data between min_height_ and max_height_
 * publish the topic_pub and that is a laserScan type 
 * only have (x,y)
 * */
class PandarTo2D
{
public:
  PandarTo2D(ros::NodeHandle n);
  ~PandarTo2D();
  void spin();
  void pointCloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
private:
  ros::NodeHandle pn_;
  ros::Subscriber pointCloud_sub_;
  ros::Publisher laserScan_pub_;
  std::string topic_sub_;
  std::string topic_pub_;
  double min_height_;
  double max_height_;
  double range_min_;
  double range_max_;
  
  double angle_min_;
  double angle_max_;
  
  double angle_increment_;
  double scan_time_;
};


// main function 
int main(int argc, char ** argv)
{
  ros::init(argc,argv,"PandarTo2D");
  ros::NodeHandle n;
  PandarTo2D *pt2;
  pt2=new PandarTo2D(n);
  pt2->spin();
  return 0;
}

PandarTo2D::PandarTo2D(ros::NodeHandle n)
{
  pn_=n;
  pn_.param<std::string>("topic_pub",topic_pub_,"/scan");
  pn_.param<std::string>("topic_sub",topic_sub_,"/pandar_node/pandar_points");
  pn_.param<double>("/pandar_node/min_hight",min_height_,-0.1);
  pn_.param<double>("/pandar_node/max_hight",max_height_,0.1);
  pn_.param<double>("/pandar_node/angle_increment", angle_increment_, M_PI / 720.0);
  pn_.param<double>("/pandar_node/scan_time", scan_time_, 1.0 / 10.0);
  pn_.param<double>("/pandar_node/angle_min", angle_min_, -3*M_PI / 4.0);
  pn_.param<double>("/pandar_node/angle_max", angle_max_, 3*M_PI / 4.0);  
  pn_.param<double>("/pandar_node/range_min", range_min_, 0.05);
  pn_.param<double>("/pandar_node/range_max", range_max_, 30.0);
  ROS_INFO("%f,%f,%f,%f",angle_min_,angle_max_,range_min_,range_max_);
  pointCloud_sub_=pn_.subscribe<sensor_msgs::PointCloud2>(topic_sub_,2,&PandarTo2D::pointCloud_cb,this);
  laserScan_pub_ =pn_.advertise<sensor_msgs::LaserScan>(topic_pub_,1);
}

PandarTo2D::~PandarTo2D()
{
}

void PandarTo2D::spin()
{
  ros::spin();
}


/*
 * @@@input cloud_msg
 * 
 * @@@publish LaserScan type
 *  */
void PandarTo2D::pointCloud_cb(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
  pcl::fromROSMsg(*cloud_msg,cloud_pcl);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_pcl,cloud_pcl,indices);
  
  ROS_INFO("size of cloud recieved : %d",int(cloud_pcl.points.size()));
  
  sensor_msgs::LaserScan output;
  
  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;
  
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
  
  output.ranges.assign(ranges_size+1, output.range_max + 1.0);
 // ROS_INFO("size is %d",ranges_size+1);
  output.header.frame_id="/laser";
  pcl::PointXYZ point;
  // ROS_INFO("123size of cloud recieved : %d",int(cloud_pcl.points.size()));
   
  for(int i=0;i<cloud_pcl.points.size();i++)
  {
    point.x=cloud_pcl.points[i].x;
    point.y=cloud_pcl.points[i].y;
    point.z=cloud_pcl.points[i].z;
     //ROS_INFO("896size of cloud recieved : %d",int(cloud_pcl.points.size()));
    if(point.z<max_height_ && point.z > min_height_)
    {
      double angle = atan2(point.y, point.x)+M_PI/2.0; 

      if(angle>M_PI)
        angle-=2*M_PI;
      if (angle < output.angle_min || angle > output.angle_max)
      {
       // ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }
      double range = hypot(point.x, point.y);
      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }
    }
    
  }
  /*
  for(sensor_msgs::PointCloud2ConstIterator<float>
		  iter_x(*cloud_msg,"x"), iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
                  iter_x != iter_x.end();
                  ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
      {
        ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_)
      {
        ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_)
      {
        ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
                      *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }

      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }
  }*/
  //ROS_INFO("size of cloud output : %d",int(output.ranges.size()));
  output.header.stamp=ros::Time::now();
  laserScan_pub_.publish(output);
  
}
