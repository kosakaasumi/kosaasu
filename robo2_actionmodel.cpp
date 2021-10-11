/**********************************************************************
   Include Libraries
**********************************************************************/
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <Eigen/Dense>

/**********************************************************************
   Declare variables(変数宣言)
**********************************************************************/
#define CYCLE_PERIOD 0.02f

std::vector<float> vx_vy_vw = {0.0f, 0.0f, 0.0f}; //xの速度、yの速度、ω
float ruto3 = 1.7320508;                          //√3
float l = 19.27;                                  //中心からホイールまでの距離
std_msgs::Float32MultiArray msg_three_speed;      //v0,v1,v2を格納 
float mata[3] = {0.0f, 0.0f, 0.0f}; 
/**********************************************************************
   Proto_type_Declare functions(受け取る関数)
**********************************************************************/
void three_speed_cb(const std_msgs::Float32MultiArray::ConstPtr &msg);
/**********************************************************************
   Main
**********************************************************************/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "actionmodel_omuni");
   ros::NodeHandle nh;
   ros::Subscriber sub_second = nh.subscribe("joy_content", 100, three_speed_cb);         //joyコンの値を受け取る
   ros::Publisher pub_float = nh.advertise<std_msgs::Float32MultiArray>("v0_v1_v2", 300); //mbedへ
   ros::Rate loop_rate(1.0f / CYCLE_PERIOD);
   msg_three_speed.data.resize(3);

   while (ros::ok())
   {
      ros::spinOnce();                                  //callback関数を読み込む  
      Eigen::Matrix<float, 3, 1> x;                     //xの速度、yの速度、ω
      x << vx_vy_vw[0],
           vx_vy_vw[1],
           vx_vy_vw[2];
      Eigen::Matrix<float, 3, 3> A;                     //運動モデルの行列式のやつ
      A << 1.0/2, -(ruto3/2),  l,
           1.0/2, ruto3/2,     l,
           1,         0,     l;
      Eigen::Matrix<float, 3, 1> result;
      result << 0,
      0,
      0;
      result = A * x;
      for(int i = 0; i < 3; i++){
          mata[i] = result(i, 0);
          msg_three_speed.data[i] = mata[i];
      }
       pub_float.publish(msg_three_speed);

       ROS_INFO("%f, %f, %f", vx_vy_vw[0],vx_vy_vw[1],vx_vy_vw[2]);

            
      loop_rate.sleep();
   }
   return 0; 
}

/**********************************************************************
   Functions <Call Back>
**********************************************************************/

void three_speed_cb(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    vx_vy_vw = msg->data;

}
