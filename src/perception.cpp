#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <ctime>
#include <fstream>
#include "ganzhi3/plane.h"
#include "ganzhi3/center.h"
#include "ganzhi3/guagou_in_houzhou.h"
#include "../include/ganzhi3/RsPointXYZIRT.h" //定义了速腾的点云结构，头文件放在devel/include中
#include "../include/ganzhi3/eul2quat.h"

using namespace std;
/*
 *此代码所有的坐标系为面向车尾时的方向为：
 * x正方向为向前
 * y正方向为向左
 * z正方向为向上
 */

ofstream ofs1;  //搜索中心
ofstream ofs2;  //欧拉角
ofstream ofs3;  //挂钩在后轴的坐标

// 全局常量定义，从参数服务器读取
double width;   //二维码板的宽度
double err_x;       //相机和雷达坐标系的x偏移
double err_y;       //相机和雷达坐标系的y偏移
double err_z;       //相机和雷达坐标系的z偏移
double samll_box_x;       //小搜索框x长度的一半
double samll_box_y;       //小搜索框y长度的一半
double samll_box_z;       //小搜索框z长度的一半
double big_box_x;            //大搜索框x长度的一半
double big_box_y;            //大搜索框y长度的一半
double big_box_z;            //大搜索框z长度的一半
int RING;                               //激光雷达线束
string search_center_file;      //搜索中心写入txt文档路径
string eul_angle_file;                //欧拉角写入txt文档路径
string guagou_in_houzhou_file;  //挂钩在后轴中心的坐标写入txt文档路径

#define pi 3.1415926
#define rear_axle_center_in_lidar_x 2.5345 //后轴中心在雷达坐标系下的x坐标
#define rear_axle_center_in_lidar_y -0.9815 //后轴中心在雷达坐标系下的y坐标
#define hook_in_erweima_x 0.765             //挂钩在二维码坐标系下的x坐标
#define hook_in_erweima_y -0.805            //挂钩在二维码坐标系下的y坐标

RsPointXYZIRT searchPoint;                    //邻域搜索圆心
ganzhi3::guagou_in_houzhou guagou_in_houzhou; //存储要发布的挂钩相对于后轴中心的坐标
ganzhi3::plane plane;                         //存储要发布的平面参数
float theta = 0.0;                            //存放二维码法向量与挂车轴线的夹角
float m1, m2, m3, m4;                         //定义全局变量用来存储平面拟合的参数

const string base_link = "kd_link"; // 搜索到的点云的坐标系
sensor_msgs::PointCloud2 cloud_pt;     //存放要发布的小邻域搜索点云
sensor_msgs::PointCloud2 cloud_ptplus; //存放要发布的大邻域搜索点云
vector<RsPointXYZIRT> POINT_in_plane;  //存放二维码平面上点云的容器

// 定义相机雷达识别到的二维码中心 全局变量
static float center_cam_x = 0.0;
static float center_cam_y = 0.0;
static float center_cam_z = 0.0;
static float center_lidar_x = 0.0;
static float center_lidar_y = 0.0;
static float center_lidar_z = 0.0;

float point_plane_distance(const RsPointXYZIRT &point);
float point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2);
void camera_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg);
void lidar_rawpoint_sub_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
void xiaolinyu_sub_to_nihe(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
void plane_para_sub_cb(const ganzhi3::plane::ConstPtr &plane);
void kdtreeplus_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
bool compare(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2);
double angle_compare(double angle);

int main(int argc, char **argv)
{
        ros::init(argc, argv, "perception");
        ros::NodeHandle nh;
        // 读取参数服务器中定义的常量
        nh.param<double> ("width", width, 0.60);   //后面的默认参数必须加，如果参数服务器没定义就会用默认值
        // nh.getParam("width", width);         //也可以用这个来获取参数，返回值是bool类型
        nh.param<double>("err_x", err_x, 0.15);
        nh.param<double> ("err_y", err_y, -0.01);
        nh.param<double> ("err_z", err_z, -0.12);
        nh.param<double> ("samll_box_x", samll_box_x, 1.5);
        nh.param<double> ("samll_box_y", samll_box_y, 0.2);
        nh.param<double> ("samll_box_z", samll_box_z, 0.25);
        nh.param<double> ("big_box_x", big_box_x, 1.5);
        nh.param<double> ("big_box_y", big_box_y, 0.5);
        nh.param<double> ("big_box_z", big_box_z, 0.4);
        nh.param<int> ("RING", RING, 80);
        nh.param<string> ("search_center_file", search_center_file, "/home/zh/catkin_ws/src/ganzhi3/result/search_center.txt");
        nh.param<string> ("eul_angle_file", eul_angle_file, "/home/zh/catkin_ws/src/ganzhi3/result/eul_angle.txt");
        nh.param<string> ("guagou_in_houzhou_file", guagou_in_houzhou_file, "/home/zh/catkin_ws/src/ganzhi3/result/guagou_in_houzhou.txt");         

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
        // 发布平面参数
        ros::Publisher plane_para_pub_ = nh.advertise<ganzhi3::plane>("/plane_para", 10);
        // 发布坐标转换后挂钩在后轴中心的坐标
        ros::Publisher transform_pub_ = nh.advertise<ganzhi3::guagou_in_houzhou>("/transform", 10);

        // 初始化搜索中心
        searchPoint.x = 0.0;
        searchPoint.y = 0.0;
        searchPoint.z = err_z;

        ofs1.open(search_center_file);
        ofs2.open(eul_angle_file);
        ofs3.open(guagou_in_houzhou_file);
        ofs1 << "center_cam_x\tcenter_cam_y\tcenter_lidar_x\tcenter_lidar_y" << endl;
        ofs2 << "raw\tpitch\tyawl\ttheta" << endl;
        ofs3 << "lidar_guagou_in_houzhou_x\tlidar_guagou_in_houzhou_y\tcam_guagou_in_houzhou_x\tcam_guagou_in_houzhou_y" << endl;

        ros::Rate loop_rate(10);
        while (ros::ok())
        {
                // time_t begin, end;
                // begin = clock();
                ros::spinOnce();

                searchPoint.x = center_cam_x;
                searchPoint.y = center_cam_y;

                ofs1 << center_cam_x << "\t" << center_cam_y << "\t" << center_lidar_x << "\t" << center_lidar_y << endl;
                cout << "搜索中心坐标:(" << searchPoint.x << ", " << searchPoint.y << ", " << searchPoint.z << ") " << endl;

                guagou_in_houzhou.guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(theta) + hook_in_erweima_x * cos(theta) + center_lidar_x - rear_axle_center_in_lidar_x;
                guagou_in_houzhou.guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(theta) - hook_in_erweima_x * sin(theta) + center_lidar_y - rear_axle_center_in_lidar_y;

                double cam_guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(theta) + hook_in_erweima_x * cos(theta) + center_cam_x - rear_axle_center_in_lidar_x;
                double cam_guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(theta) - hook_in_erweima_x * sin(theta) + center_cam_y - rear_axle_center_in_lidar_y;

                ofs3 << guagou_in_houzhou.guagou_in_houzhouzhongxin_x << "\t" << guagou_in_houzhou.guagou_in_houzhouzhongxin_y << "\t" << cam_guagou_in_houzhouzhongxin_x << "\t" << cam_guagou_in_houzhouzhongxin_y << endl;
                kdtree_pc_pub_.publish(cloud_pt);          //发布小邻域搜索到的点云
                kdtreeplus_pc_pub_.publish(cloud_ptplus);  //发布大邻域搜索到的点云
                plane_para_pub_.publish(plane);            //发布平面参数
                transform_pub_.publish(guagou_in_houzhou); //发布坐标转换后挂钩在后轴中心的坐标
                // end = clock();
                // cout << "while循环用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
                loop_rate.sleep();
        }
        ofs1.close(); //关闭文件流
        ofs2.close(); //关闭文件流
        ofs3.close(); //关闭文件流

        return 0;
}

// 相机订阅
void camera_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg)
{
        center_cam_x = (cloud_msg->pose.position.z) + err_x;  // 相机的z方向是雷达的x方向
        center_cam_y = -(cloud_msg->pose.position.x) + err_y; //相机的-x方向是雷达的+y方向
        center_cam_z = err_z;
        // cout << "相机识别二维码中心："  << "(x = " << center_cam_x << ", y = " << center_cam_y << ", z = " << center_cam_z << " )" << endl;
        Eigen::Vector3d temp_eulur;
        temp_eulur = ToEulur(Eigen::Quaterniond(cloud_msg->pose.orientation.w, cloud_msg->pose.orientation.x, cloud_msg->pose.orientation.y, cloud_msg->pose.orientation.z));
        // cout << "pitch = " << 57.296 * temp_eulur[1] << endl;
        double raw = 57.296 * temp_eulur[0];
        double pitch = 57.296 * temp_eulur[1];
        double yawl = 57.296 * temp_eulur[2];
        raw = angle_compare(raw);
        pitch = angle_compare(pitch);
        yawl = angle_compare(yawl);

        ofs2 << raw << "\t" << pitch << "\t" << yawl << "\t" << 57.296 * theta << endl;
        return;
} // 相机订阅

// 小邻域搜索
void small_search(pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud)
{
        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);                                   //存储小邻域搜索后的点云
        pcl::CropBox<RsPointXYZIRT> box_filter;                                                                                 //滤波器对象
        box_filter.setMin(Eigen::Vector4f(searchPoint.x - samll_box_x, searchPoint.y - samll_box_y, err_z - samll_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
        box_filter.setMax(Eigen::Vector4f(searchPoint.x + samll_box_x, searchPoint.y + samll_box_y, err_z + samll_box_z, 1.0));
        box_filter.setNegative(false);            // false是将盒子内的点去除，默认为false
        box_filter.setInputCloud(original_cloud); //输入源
        box_filter.filter(*cloud_kdtree);         //保留！
        pcl::toROSMsg(*cloud_kdtree, cloud_pt);   //把小邻域搜索到的点云转换成ROS的发布格式
        cloud_pt.header.stamp = ros::Time::now();
        // cloud_pt.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
        return;
} // 小邻域搜索

// 大邻域搜索
void big_search(pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud)
{
        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>);                             //存储大邻域搜索后的点云
        pcl::CropBox<RsPointXYZIRT> boxplus_filter;                                                                           //滤波器对象
        boxplus_filter.setMin(Eigen::Vector4f(searchPoint.x - big_box_x, searchPoint.y - big_box_y, err_z - big_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
        boxplus_filter.setMax(Eigen::Vector4f(searchPoint.x + big_box_x, searchPoint.y + big_box_y, err_z + big_box_z, 1.0));
        boxplus_filter.setNegative(false);              // false是将盒子内的点去除，默认为false
        boxplus_filter.setInputCloud(original_cloud);   //输入源
        boxplus_filter.filter(*cloud_kdtreeplus);       //保留！
        pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式
        cloud_ptplus.header.stamp = ros::Time::now();
        cloud_ptplus.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中
        return;
} // 大邻域搜索

// 大小邻域搜索
void lidar_rawpoint_sub_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        pcl::PointCloud<RsPointXYZIRT>::Ptr original_cloud(new pcl::PointCloud<RsPointXYZIRT>); //存放雷达原始点云
        pcl::fromROSMsg(*cloud_msg, *original_cloud);   //把雷达原始点云从ROS格式转换成RsPointXYZIRT格式并存放到original_cloud里

        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtree(new pcl::PointCloud<RsPointXYZIRT>);                                   //存储小邻域搜索后的点云
        pcl::CropBox<RsPointXYZIRT> box_filter;                                                                                 //滤波器对象
        box_filter.setMin(Eigen::Vector4f(searchPoint.x - samll_box_x, searchPoint.y - samll_box_y, err_z - samll_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
        box_filter.setMax(Eigen::Vector4f(searchPoint.x + samll_box_x, searchPoint.y + samll_box_y, err_z + samll_box_z, 1.0));
        box_filter.setNegative(false);            // false是将盒子内的点去除，默认为false
        box_filter.setInputCloud(original_cloud); //输入源
        box_filter.filter(*cloud_kdtree);         //保留！
        pcl::toROSMsg(*cloud_kdtree, cloud_pt);   //把小邻域搜索到的点云转换成ROS的发布格式
        cloud_pt.header.stamp = ros::Time::now();
        cloud_pt.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中

        pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_kdtreeplus(new pcl::PointCloud<RsPointXYZIRT>);                             //存储大邻域搜索后的点云
        pcl::CropBox<RsPointXYZIRT> boxplus_filter;                                                                           //滤波器对象
        boxplus_filter.setMin(Eigen::Vector4f(searchPoint.x - big_box_x, searchPoint.y - big_box_y, err_z - big_box_z, 1.0)); // Min和Max是指立方体的两个对角点。每个点由一个四维向量表示，齐次坐标）
        boxplus_filter.setMax(Eigen::Vector4f(searchPoint.x + big_box_x, searchPoint.y + big_box_y, err_z + big_box_z, 1.0));
        boxplus_filter.setNegative(false);              // false是将盒子内的点去除，默认为false
        boxplus_filter.setInputCloud(original_cloud);   //输入源
        boxplus_filter.filter(*cloud_kdtreeplus);       //保留！
        pcl::toROSMsg(*cloud_kdtreeplus, cloud_ptplus); //把大邻域搜索到的点云转换成ROS的发布格式
        cloud_ptplus.header.stamp = ros::Time::now();
        cloud_ptplus.header.frame_id = base_link; //设置发布点云的坐标系：要设置在输出点云存储的容器中

        return;
} //大小邻域搜索

// 拟合平面
void xiaolinyu_sub_to_nihe(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        // time_t begin, end;
        // begin = clock();
        pcl::PointCloud<RsPointXYZIRT>::Ptr kdtree_cloud(new pcl::PointCloud<RsPointXYZIRT>); // 存放小邻域搜索到的点云
        pcl::fromROSMsg(*cloud_msg, *kdtree_cloud);     // 把ROS的点云格式转换成PCL格式
        int PointsNumber_kdtree_cloud = kdtree_cloud->size();   // 计算小邻域搜索的点云数

        if (PointsNumber_kdtree_cloud > 20)     //如果点数大于20，进行拟合平面
        {
                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //创建一个模型参数对象，用于记录拟合结果
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // inliers通过点云序号来记录模型内点
                pcl::SACSegmentation<RsPointXYZIRT> seg;        //创建一个分割对象
                seg.setOptimizeCoefficients(true); // flase 展示的是分割剩下的点
                // Mandatory-设置目标几何形状
                seg.setModelType(pcl::SACMODEL_PLANE);

                //分割方法：随机采样法
                seg.setMethodType(pcl::SAC_RANSAC);
                //设置误差容忍范围
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
                theta = plane.jiajiao = atan(plane.b / plane.a);
        }
        // end = clock();
        // cout << "拟合平面用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
} // 拟合平面

// 平面参数订阅
void plane_para_sub_cb(const ganzhi3::plane::ConstPtr &plane)
{
        m1 = plane->a;
        m2 = plane->b;
        m3 = plane->c;
        m4 = plane->d;
        return;
} // 平面参数订阅

// 雷达计算二维码中心
void kdtreeplus_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
        clock_t begin, end;
        begin = clock();
        pcl::PointCloud<RsPointXYZIRT>::Ptr kdtreeplus_pointcloud(new pcl::PointCloud<RsPointXYZIRT>); //存储大邻域传进来的点云
        pcl::fromROSMsg(*cloud_msg, *kdtreeplus_pointcloud);

        std::vector<pcl::PointCloud<RsPointXYZIRT>> laserRing(RING); //定义雷达线束，动态数组集合
        int cloudsize = (*kdtreeplus_pointcloud).points.size();      //大邻域搜索到的点云数
        for (int i = 0; i < cloudsize; ++i)
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
                                if (plane_distance > -0.08 && plane_distance < 0.08) //点到平面的距离阈值设为5cm
                                {
                                        POINT_in_plane.push_back(laserRing[i].points[j]);
                                }
                        }

                        if (!POINT_in_plane.empty())
                        {
                                int number_POINT_in_plane = POINT_in_plane.size(); //平面上的点数
                                point_distance = point_point_distance(POINT_in_plane[1], POINT_in_plane[number_POINT_in_plane - 1]);
                                if (point_distance > width - 0.1 && point_distance < width + 0.1)
                                {
                                        average_X += (POINT_in_plane[1].x + POINT_in_plane[number_POINT_in_plane - 1].x) / 2;
                                        average_Y += (POINT_in_plane[1].y + POINT_in_plane[number_POINT_in_plane - 1].y) / 2;
                                        calculate_N++;
                                }
                        }
                        POINT_in_plane.clear(); // 清空数组
                }
        }
        float temp_x = center_lidar_x;
        float temp_y = center_lidar_y;
        center_lidar_x = average_X / calculate_N;
        center_lidar_y = average_Y / calculate_N;
        if (center_lidar_x != center_lidar_x){
                center_lidar_x = temp_x;
                center_lidar_y = temp_y;
        }

        if (calculate_N > 0)
        {
                cout << "二维码平面上总共有：" << calculate_N << "条激光雷达点云线束" << endl;
                cout << "二维码中心雷达计算坐标为：(" << center_lidar_x << "," << center_lidar_y << ")" << endl;
        }
        end = clock();
        // cout << "雷达计算二维码中心用时:" << double(end - begin) / CLOCKS_PER_SEC * 1000 << endl;
        return;
} // 雷达计算二维码中心

// 点到平面距离函数实现
float point_plane_distance(const RsPointXYZIRT &point)
{
        float fenzi = m1 * point.x + m2 * point.y + m3 * point.z + m4;
        float fenmu = sqrt(m1 * m1 + m2 * m2 + m3 * m3);
        float point_to_plane_dist = fenzi / fenmu;
        return point_to_plane_dist;
} // 点到平面距离函数实现

// 点到点距离函数实现
float point_point_distance(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2)
{
        float point_to_point_dist = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z));
        return point_to_point_dist;
} // 点到点距离函数实现

// 点的排序方式
bool compare(const RsPointXYZIRT &point1, const RsPointXYZIRT &point2)
{
        return point1.y < point2.y;
} // 点的排序方式

double angle_compare(double angle){
        if(angle > 90.0)
                return angle - 180;
        else if(angle < -90.0)
                return angle + 180;
        return -angle;
}