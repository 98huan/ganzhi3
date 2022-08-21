#ifndef GLOBLE_CECNTER_H
#define GLOBLE_CECNTER_H
#include "ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构，头文件放在devel/include中

// 从相机得到二维码中心
float center_cam_x;
float center_cam_y;
float center_cam_z;

// 从雷达得到二维码中心
float center_lidar_x;
float center_lidar_y;
float center_lidar_z;

RsPointXYZIRT  searchPoint;						//邻域搜索圆心 

#endif