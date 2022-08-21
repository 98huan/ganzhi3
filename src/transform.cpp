#include <iostream>
#include <math.h>
using namespace std;
// 测试用坐标转换
#define rear_axle_center_in_lidar_x            2.5345                 //后轴中心在雷达坐标系中的位姿 
#define rear_axle_center_in_lidar_y            -0.9815               // 后轴中心在雷达坐标系中的位姿
#define hook_in_erweima_x                           0.765               // 挂钩在二维码坐标系中的位姿
#define hook_in_erweima_y                           -0.805              // 挂钩在二维码坐标系中的位姿

float guagou_in_houzhouzhongxin_x;        // 挂钩在后轴中心坐标系下的位姿
float guagou_in_houzhouzhongxin_y;        // 挂钩在后轴中心坐标系下的位姿

void coordinate_transformation(float theta, float center_c_x, float center_c_y){
   guagou_in_houzhouzhongxin_x = hook_in_erweima_y * sin(theta) + hook_in_erweima_x * cos(theta)
                                                                      + center_c_x
                                                                      - rear_axle_center_in_lidar_x;
   guagou_in_houzhouzhongxin_y = hook_in_erweima_y * cos(theta) - hook_in_erweima_x * sin(theta)
                                                                      + center_c_y 
                                                                      - rear_axle_center_in_lidar_y;
}

int main(){
   coordinate_transformation(0, 0, 0);
   cout << guagou_in_houzhouzhongxin_x << endl;
   cout << guagou_in_houzhouzhongxin_y << endl;

   return 0;
}
