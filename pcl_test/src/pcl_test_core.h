#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT -0.5 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18//雷达自身参数
#define SENSOR_HEIGHT 1.78//定义传感器高度

#define concentric_divider_distance_ 0.01 //0.1 meters default 
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree 设定的同条射线上邻近两点的坡度阈值
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree 整个地面的坡度阈值,单位为度。
#define reclass_distance_threshold_ 0.2

class PclTestCore
{

private:
  ros::Subscriber sub_point_cloud_; //为接收点云信息创建了一个订阅节点

  ros::Publisher pub_ground_, pub_no_ground_; //创建了两个发布滤波的节点，其中一个是测试节点

  struct PointXYZIRTColor
  {
    pcl::PointXYZI point;

    float radius; //cylindric coords on XY Plane : XY平面上的圆柱坐标
    float theta;  //angle deg on XY plane : XY平面上的角度

    size_t radial_div;     //index of the radial divsion to which this point belongs to 此点所属的径向分区的索引
    size_t concentric_div; //index of the concentric division to which this points belongs to 此点所属的同心分区索引

    size_t original_index; //index of this point in the source pointcloud 源点云中该点的索引
  };



  //为了方便地对点进行半径和夹角的表示，我们使用如下数据结构代替pcl::PointCloudXYZI：
  typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor; //将std::vector<PointXYZIRTColor>更名为PointCloudXYZIRTColor

  size_t radial_dividers_num_;
  size_t concentric_dividers_num_;

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);//切除函数声明

  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);//切除函数声明

  void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                        PointCloudXYZIRTColor &out_organized_points,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

  void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices);

  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);



public:
  PclTestCore(ros::NodeHandle &nh);
  ~PclTestCore();
  void Spin();
};

//1、public修饰的成员变量
//在程序的任何地方都可以被访问，就是公共变量的意思，不需要通过成员函数就可以由类的实例直接访问
//2、private修饰的成员变量
//只有类内可直接访问，私有的，类的实例要通过成员函数才可以访问，这个可以起到信息隐藏
