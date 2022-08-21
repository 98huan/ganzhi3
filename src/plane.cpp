#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include "ganzhi3/plane.h"
#include "/home/zh/catkin_ws/src/ganzhi3/include/ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构，头文件放在devel/include中

using namespace std;

#define pi 3.1415926
/*
    功能：
	拟合平面，发布前表面法向量theta角
*/

ganzhi3::plane plane; //实例化一个plane的msg格式
// float A, B, C, D;
typedef RsPointXYZIRT PointType;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
	// 存放小邻域搜索到的点云
	pcl::PointCloud<RsPointXYZIRT>::Ptr kdtree_cloud(new pcl::PointCloud<RsPointXYZIRT>);
	// 把ROS的点云格式转换成PCL格式
	pcl::fromROSMsg(*cloud_msg, *kdtree_cloud);

	// 计算小邻域搜索的点云数
	int PointsNumber_kdtree_cloud = kdtree_cloud->size();
	cout << "小邻域搜得的点云数：" << PointsNumber_kdtree_cloud << "     ";
	/************** 如果点数大于20，进行拟合平面****************************/
	if (PointsNumber_kdtree_cloud > 20)
	{
		//创建一个模型参数对象，用于记录拟合结果
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		// inliers通过点云序号来记录模型内点
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		//创建一个分割对象
		pcl::SACSegmentation<RsPointXYZIRT> seg;
		// Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
		seg.setOptimizeCoefficients(true); // flase 展示的是分割剩下的点
		// Mandatory-设置目标几何形状
		seg.setModelType(pcl::SACMODEL_PLANE);
		//分割方法：随机采样法
		seg.setMethodType(pcl::SAC_RANSAC);
		//设置误差容忍范围，也就是阈值
		seg.setDistanceThreshold(0.03);
		// 设置最大迭代次数
		seg.setMaxIterations(500);
		//输入点云
		seg.setInputCloud(kdtree_cloud);
		//分割点云
		seg.segment(*inliers, *coefficients);

		// 输出4个平面参数
		plane.a = coefficients->values[0]; //赋值给msg里面的平面参数
		plane.b = coefficients->values[1]; //赋值给msg里面的平面参数
		plane.c = coefficients->values[2]; //赋值给msg里面的平面参数
		plane.d = coefficients->values[3]; //赋值给msg里面的平面参数

		plane.jiajiao = atan(plane.b / plane.a);
		cout << "theta = " << 180 * atan(plane.b / plane.a) / pi << "度" << endl; // theta
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane");
	ros::NodeHandle nh;
	ros::Publisher plane_para_publisher_; // 平面参数发布者
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/output_kdtree", 1, cloud_cb);
	plane_para_publisher_ = nh.advertise<ganzhi3::plane>("/plane_para", 10);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		plane_para_publisher_.publish(plane);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}