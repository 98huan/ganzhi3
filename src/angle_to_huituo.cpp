#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "/home/zh/catkin_ws/src/ganzhi3/include/ganzhi3/eul2quat.h"
#include "ganzhi3/angle_to_huituo.h"
using namespace std;
/*
        功能：发布转角给慧拓
*/

ganzhi3::angle_to_huituo angle; //实例化发布给慧拓的角度

double angle_compare(double angle){
        if(angle > 90.0)
                return angle - 180;
        else if(angle < -90.0)
                return angle + 180;
        return -angle;
}
// 订阅相机发布的二维码中心
void quaterniond_sub_cb(const geometry_msgs::PoseStampedConstPtr &cloud_msg2)
{
        Eigen::Vector3d temp_eulur;
        temp_eulur = ToEulur(Eigen::Quaterniond(cloud_msg2->pose.orientation.w, cloud_msg2->pose.orientation.x, cloud_msg2->pose.orientation.y, cloud_msg2->pose.orientation.z));
        angle.angle = angle_compare(57.296 * temp_eulur[2]) - 0.55;
        cout << "yaw = " << angle.angle << endl;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "angle_to_huituo");
        ros::NodeHandle nh;
        ros::Subscriber quaterniond_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 1, quaterniond_sub_cb);       //订阅相机发布的四元数	订阅方

        ros::Publisher angle_pub_ = nh.advertise<ganzhi3::angle_to_huituo>("/angle_to_huituo", 1);      //发布转角给慧拓

        ros::Rate loop_rate(100); //循环发布频率为100HZ
        while (ros::ok())
        {
                ros::spinOnce();
                angle_pub_.publish(angle);      
                loop_rate.sleep();
        }
        return 0;
}
