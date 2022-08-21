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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <ctime>

#include "ganzhi3/plane.h"
#include "ganzhi3/center.h"
#include "ganzhi3/guagou_in_houzhou.h"
#include "../include/ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构，头文件放在devel/include中
#include "../include/ganzhi3/eul2quat.h"

using namespace std;
#define width 0.60            //二维码板的宽度
#define err_x 0.15            //相机雷达x偏移
#define err_y 0.071           //相机雷达y偏移
const float radius = 0.4;     //小邻域搜索半径
const float radiusplus = 1.0; //大邻域搜索半径

#define pi 3.1415926
#define RING 80
#define rear_axle_center_in_lidar_x 2.5345
#define rear_axle_center_in_lidar_y -0.9815
#define hook_in_erweima_x 0.765
#define hook_in_erweima_y -0.805

ganzhi3::plane plane;                         //实例化一个plane的msg格式
static RsPointXYZIRT searchPoint;             //邻域搜索圆心
sensor_msgs::PointCloud2 cloud_pt;            //存放要发布的小邻域搜索点云
sensor_msgs::PointCloud2 cloud_ptplus;        //存放要发布的大邻域搜索点云
ganzhi3::center center;                       //存储要发布的二维中心坐标
ganzhi3::guagou_in_houzhou guagou_in_houzhou; //存储要发布的挂钩相对于后轴中心的坐标
vector<RsPointXYZIRT> POINT_in_plane;         //存放平面上点云的容器

const string base_link = "kd_link"; // kdtree搜索的坐标系
// 定义相机雷达识别的二维码中心 全局变量
static float center_cam_x = 0.0;
static float center_cam_y = 0.0;
static float center_cam_z = -0.12;
static float center_lidar_x = 0.0;
static float center_lidar_y = 0.0;
static float center_lidar_z = -0.12;

static float camera_x = 0.0; //存放相机识别的二维码中心坐标
static float camera_y = 0.0; //存放相机识别的二维码中心坐标
static float lidar_x = 0.0;  //存放雷达计算的二维码中心坐标
static float lidar_y = 0.0;  //存放雷达计算的二维码中心坐标
static float theta = 0.0;    //存放二维码法向量与挂车轴线的夹角
//定义全局变量用来存储平面拟合的参数
float m1, m2, m3, m4, jiajiao;

bool compare(RsPointXYZIRT point1, RsPointXYZIRT point2)
{
        return point1.y < point2.y;
}

float point_plane_distance(const RsPointXYZIRT &point);
float point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2);
void camera_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg);
void lidar_rawpoint_sub_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
void xiaolinyu_sub_to_nihe(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
void plane_para_sub_cb(const ganzhi3::plane::ConstPtr &plane);
void kdtreeplus_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

int main(int argc, char **argv)
{
        ros::init(argc, argv, "plane");
        ros::NodeHandle nh;
        // 订阅相机发布的二维码点坐标
        ros::Subscriber camera_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, camera_sub_cb);
        // 订阅雷达原始点云,发布大小邻域搜索点云
        ros::Subscriber lidar_rawpoint_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, lidar_rawpoint_sub_cb);
        // 订阅小邻域搜索到的点云,拟合平面
        ros::Subscriber xiaolinyu_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/output_kdtree", 10, xiaolinyu_sub_to_nihe);
        // 订阅平面参数
        ros::Subscriber plane_para_sub_ = nh.subscribe<ganzhi3::plane>("/plane_para", 10, plane_para_sub_cb);
        // 订阅到大邻域搜索到的点云后，进入回调函数进行计算
        ros::Subscriber kdtreeplus_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>("/output_kdtreeplus", 10, kdtreeplus_subscriber_cb);

        // 发布小邻域搜索点云
        ros::Publisher kdtree_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtree", 10);
        // 发布大邻域搜索点云
        ros::Publisher kdtreeplus_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/output_kdtreeplus", 10);
        // 平面参数发布者
        ros::Publisher plane_para_publisher_ = nh.advertise<ganzhi3::plane>("/plane_para", 10);
        ros::Publisher transform_pub_ = nh.advertise<ganzhi3::guagou_in_houzhou>("/transform", 10);
        ros::Publisher center_pub = nh.advertise<ganzhi3::center>("/search_center", 10);

        searchPoint.x = 0.0;
        searchPoint.y = 0.0;

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
                time_t begin, end;
                begin = clock();
                if (searchPoint.x < 4 || center_cam_x < 6)
                {
                        center.c_x = center_cam_x;
                        center.c_y = center_cam_y;
                        center.c_theta = 0;
                }
                else if (lidar_x != lidar_x) //雷达计算为nan时继续上次的数据
                {
                        center.c_x = center.c_x;
                        center.c_y = center.c_y;
                        center.c_theta = center.c_theta;
                }
                else
                {
                        center.c_x = lidar_x;
                        center.c_y = lidar_y;
                        center.c_theta = jiajiao;
                }
                searchPoint.x = center_lidar_x = center.c_x;
                searchPoint.y = center_lidar_y = center.c_y;
                searchPoint.z = center_lidar_z = center_cam_z;
                cout << "搜索中心坐标:(" << searchPoint.x << ", " << searchPoint.y << ", " << searchPoint.z << ") " << endl;

                guagou_in_houzhou.guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(center.c_theta) + hook_in_erweima_x * cos(center.c_theta) + searchPoint.x - rear_axle_center_in_lidar_x;
                guagou_in_houzhou.guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(center.c_theta) - hook_in_erweima_x * sin(center.c_theta) + searchPoint.y - rear_axle_center_in_lidar_y;

                kdtree_pc_pub_.publish(cloud_pt);          //发布小邻域搜索到的点云
                kdtreeplus_pc_pub_.publish(cloud_ptplus);  //发布大邻域搜索到的点云
                plane_para_publisher_.publish(plane);      //发布平面参数
                center_pub.publish(center);                // 发布计算得到的二维中心坐标
                transform_pub_.publish(guagou_in_houzhou); //发布坐标转换后挂钩在后轴中心的坐标
                ros::spinOnce();
                end = clock();
                cout << "while循环用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000  << "ms" << endl;
                loop_rate.sleep();
        }
        return 0;
}

void camera_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg)
{
        time_t begin, end;
        begin = clock();
        center_cam_x = (cloud_msg->pose.position.z) + err_x;  // 相机的z方向是雷达的x方向
        center_cam_y = -(cloud_msg->pose.position.x) + err_y; //相机的-x方向是雷达的+y方向
        center_cam_z = -0.12;                                 //相机的-y方向是雷达的+z方向
        // cout << "相机识别二维码中心："  << "(x = " << center_cam_x << ", y = " << center_cam_y << ", z = " << center_cam_z << " )" << endl;
        end = clock();
        // cout << "订阅相机用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
}

// 大小邻域搜索
void lidar_rawpoint_sub_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        time_t begin, end;
        begin = clock();
        // pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud(new pcl::PointCloud<RsPointXYZIRT>);   //存放雷达原始点云
        // pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);     //存储小邻域搜索后的点云
        // pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域搜索后的点云
        // // sensor_msgs::PointCloud2 cloud_pt;      //存放要发布的小邻域搜索点云
        // // sensor_msgs::PointCloud2 cloud_ptplus;          //存放要发布的大邻域搜索点云
        // pcl::fromROSMsg(*cloud_msg, *original_cloud);          //把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里
        // //创建kdtree 结构
        // pcl::KdTreeFLANN<RsPointXYZIRT> kdtree;
        // pcl::KdTreeFLANN<RsPointXYZIRT> kdtreeplus;
        // //传入原始点云用于邻域搜索
        // kdtree.setInputCloud(original_cloud);
        // kdtreeplus.setInputCloud(original_cloud);
        // // 创建两个向量，分别存放近邻的索引值、近邻的中心距
        // std::vector<int> pointIdxRadiusSearch;              //索引值
        // std::vector<float> pointRadiusSquaredDistance;      //中心距
        // std::vector<int> pointIdxRadiusSearch_plus;         //索引值
        // std::vector<float> pointRadiusSquaredDistance_plus; //中心距
        // cloud_kdtree->header.frame_id = base_link;     //设置发布点云的坐标系：要设置在输出点云存储的容器中
        // cloud_kdtreeplus->header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
        // int num1 = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        // int num2 = kdtreeplus.radiusSearch(searchPoint, radiusplus, pointIdxRadiusSearch_plus, pointRadiusSquaredDistance_plus);

        pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud(new pcl::PointCloud<RsPointXYZIRT>);   //存放雷达原始点云
        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);     //存储小邻域搜索后的点云
        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域搜索后的点云
        pcl::fromROSMsg(*cloud_msg, *original_cloud);                                             //把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里
        pcl::CropBox<RsPointXYZIRT> box_filter;                                                   //滤波器对象
        pcl::CropBox<RsPointXYZIRT> boxplus_filter;                                               //滤波器对象

        box_filter.setMin(Eigen::Vector4f(searchPoint.x - 1.0, searchPoint.y - 0.3, -0.35, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.（不知道为什么要有四个，大神知道的给解答一下下）
        box_filter.setMax(Eigen::Vector4f(searchPoint.x + 1.0, searchPoint.y + 0.3, 0.1, 1.0));

        boxplus_filter.setMin(Eigen::Vector4f(searchPoint.x - 1.5, searchPoint.y - 0.6, -0.5, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，通常最后一个是1.（不知道为什么要有四个，大神知道的给解答一下下）
        boxplus_filter.setMax(Eigen::Vector4f(searchPoint.x + 1.5, searchPoint.y + 0.6, 0.4, 1.0));

        box_filter.setNegative(false);            //是保留立方体内的点而去除其他点，还是反之。false是将盒子内的点去除，默认为false
        box_filter.setInputCloud(original_cloud); //输入源
        box_filter.filter(*cloud_kdtree);         //保留！

        boxplus_filter.setNegative(false);              //是保留立方体内的点而去除其他点，还是反之。false是将盒子内的点去除，默认为false
        boxplus_filter.setInputCloud(original_cloud);   //输入源
        boxplus_filter.filter(*cloud_kdtreeplus);       //保留！
        pcl::toROSMsg(*cloud_kdtree, cloud_pt);         //把小邻域搜索到的点云转换成ROS的发布格式
        pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式

        cloud_pt.header.stamp = ros::Time::now();
        cloud_ptplus.header.stamp = ros::Time::now();
        cloud_pt.header.frame_id = base_link;     //设置发布点云的坐标系：要设置在输出点云存储的容器中
        cloud_ptplus.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
        end = clock();
        // cout << "大小邻域搜索用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
} //大小邻域搜索

// 拟合平面
void xiaolinyu_sub_to_nihe(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        time_t begin, end;
        begin = clock();
        pcl::PointCloud<RsPointXYZIRT>::Ptr kdtree_cloud(new pcl::PointCloud<RsPointXYZIRT>); // 存放小邻域搜索到的点云
        pcl::fromROSMsg(*cloud_msg, *kdtree_cloud);                                           // 把ROS的点云格式转换成PCL格式
        int PointsNumber_kdtree_cloud = kdtree_cloud->size();                                 // 计算小邻域搜索的点云数

        if (PointsNumber_kdtree_cloud > 20)
        {                                                                             //如果点数大于20，进行拟合平面
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //创建一个模型参数对象，用于记录拟合结果
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // inliers通过点云序号来记录模型内点
                pcl::SACSegmentation<RsPointXYZIRT> seg;                              //创建一个分割对象
                // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
                seg.setOptimizeCoefficients(true); // flase 展示的是分割剩下的点
                // Mandatory-设置目标几何形状
                seg.setModelType(pcl::SACMODEL_PLANE);

                //分割方法：随机采样法
                seg.setMethodType(pcl::SAC_RANSAC);
                //设置误差容忍范围，也就是阈值
                seg.setDistanceThreshold(0.03);
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
                // cout << "theta = " << 180 * plane.jiajiao / pi << endl; // theta
        }
        end = clock();
        // cout << "拟合平面用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
} //拟合平面

// 平面参数订阅
void plane_para_sub_cb(const ganzhi3::plane::ConstPtr &plane)
{
        m1 = plane->a;
        m2 = plane->b;
        m3 = plane->c;
        m4 = plane->d;
        jiajiao = plane->jiajiao;
        return;
}

// 雷达计算二维码中心
void kdtreeplus_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        clock_t begin, end;
        begin = clock();
        pcl::PointCloud<RsPointXYZIRT>::Ptr kdtreeplus_pointcloud(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域传进来的点云
        pcl::fromROSMsg(*cloud_msg, *kdtreeplus_pointcloud);

        std::vector<pcl::PointCloud<RsPointXYZIRT>> laserRing(RING); //定义雷达线束，动态数组集合
        int cloudsize = (*kdtreeplus_pointcloud).points.size();      //大邻域搜索到的点云数
        for (int i = 0; i < cloudsize; i++)
        {
                laserRing[kdtreeplus_pointcloud->points[i].ring].push_back(kdtreeplus_pointcloud->points[i]); // 将每个点放入对应的帧通道中
        }

        float average_X = 0.0, average_Y = 0.0; //二维码平均中心坐标
        int calculate_N = 0;                    //记录二维码平面上的激光线束
        for (int i = 0; i < RING; ++i)          // RING个通道逐一遍历
        {
                if (!laserRing[i].empty()) //判断该通道中是否有点云
                {
                        // 给该通道中的所有点按水平旋转角从小到大排列
                        sort(laserRing[i].begin(), laserRing[i].end(), compare);

                        float plane_distance; //点到平面的距离
                        float point_distance; //用两个边点的距离表示棱的距离
                        int laserRing_num = laserRing[i].size();
                        for (int j = 0; j < laserRing_num; ++j) //计算点到平面的距离
                        {
                                //  调用点到平面距离计算公式，计算每个点到平面的距离是否在阈值内来判断是否是平面点
                                plane_distance = point_plane_distance(laserRing[i].points[j]);
                                //判断点是否在平面上
                                if (plane_distance > -0.05 && plane_distance < 0.05) //点到平面的距离阈值设为5cm
                                {
                                        POINT_in_plane.push_back(laserRing[i].points[j]);
                                }
                        }

                        if (!POINT_in_plane.empty())
                        {
                                int number_POINT_in_plane = POINT_in_plane.size(); //平面上的点数
                                point_distance = point_point_distance(POINT_in_plane[1], POINT_in_plane[number_POINT_in_plane - 1]);
                                // cout << "棱长为:" << point_distance << endl;
                                if (point_distance > width - 0.05 && point_distance < width + 0.05)
                                {
                                        average_X += (POINT_in_plane[1].x + POINT_in_plane[number_POINT_in_plane - 1].x) / 2;
                                        average_Y += (POINT_in_plane[1].y + POINT_in_plane[number_POINT_in_plane - 1].y) / 2;
                                        calculate_N++;
                                }
                        }
                        POINT_in_plane.clear(); // 清空数组
                }
        }
        lidar_x = average_X / calculate_N;
        lidar_y = average_Y / calculate_N;

        if (calculate_N != 0)
        {
                cout << "二维码平面上总共有：" << calculate_N << "条激光雷达点云线束" << endl;
                cout << "二维码中心雷达计算坐标为：(" << lidar_x << "," << lidar_y << ")" << endl;
        }
        end = clock();
        // cout << "雷达计算二维码中心用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
}

// 点到平面距离函数实现
float point_plane_distance(const RsPointXYZIRT &point)
{
        float fenzi = m1 * point.x + m2 * point.y + m3 * point.z + m4;
        float fenmu = sqrt(m1 * m1 + m2 * m2 + m3 * m3);
        float point_to_plane_dist = fenzi / fenmu;
        return point_to_plane_dist;
}

// 点到点距离函数实现
float point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2)
{
        float point_to_point_dist = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z));
        return point_to_point_dist;
}
