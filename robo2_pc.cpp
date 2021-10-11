/*********************************
回収機構と発射機構の制御プログラム(test)
**********************************/
/**********************************************************************
Include Libraries
**********************************************************************/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <vector>
/**********************************************************************
Declare MACRO
**********************************************************************/
#define CTRL_PERIOD 0.02f
/**********************************************************************
Proto_type_Declare functions
**********************************************************************/
void joy_cd(const std_msgs::Int32MultiArray::ConstPtr &msg);
void if_retrieval_launch();
/**********************************************************************
Declare variables
**********************************************************************/
int cylinder1_move = 0; //エアシリンダー1の伸び縮みを格納(伸びているとき1,縮んでいるとき0)
int count = 0;          //エアシリンダー1の2秒待つときのカウント
int servo1_angle = 0;   //サーボモーター1の角度を格納(0°の時0,180°の時1)
int cylinder2_move = 0; //エアシリンダー2の伸び縮みを格納(伸びているとき1,縮んでいるとき0)

std::vector<int> cylinder_servo = {0, 0, 0}; //送られてきたやつを格納,シリンダー１、シリンダー２、サーボモーター１
std_msgs::Float32MultiArray msg_float;
/**********************************************************************
Main
**********************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo2_pc");
    ros::NodeHandle n;
    ros::Publisher pub_cylinder_servo = n.advertise<std_msgs::Float32MultiArray>("robo_retrieve_launch", 100);
    ros::Subscriber sub_joy = n.subscribe("joy_retrieve_launch", 100, joy_cd);
    ros::Rate loop_rate(1 / CTRL_PERIOD);
    msg_float.data.resize(3);

    while (ros::ok())
    {
        ros::spinOnce();
        if_retrieval_launch();
        msg_float.data[0] = cylinder1_move;     //発射機構のシリンダー
        msg_float.data[1] = cylinder2_move;     //回収機構のシリンダー
        msg_float.data[2] = 180 * servo1_angle; //回収機構のサーボモーター
        pub_cylinder_servo.publish(msg_float);

        //ROS_INFO("%i, %i, %i", cylinder_servo[0],cylinder_servo[1],cylinder_servo[2]);

        loop_rate.sleep();
    }
}
/*****************************************************************
Functions <Call Back>
*****************************************************************/
void joy_cd(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    cylinder_servo = msg->data;
}

void if_retrieval_launch()
{
    //発射機構のエアシリンダーの制御
    if (cylinder_servo[0])
    {
        cylinder1_move = 1;
    }
    if (cylinder1_move)
    {
        count++;
    }
    //プログラムは50Hzで動いている、2秒待つことはcount = 100になる時
    if (count == 100)
    {
        cylinder1_move = 0;
        count = 0;
    }

    //回収機構のエアシリンダーの制御
    if (cylinder_servo[1] && cylinder2_move == 0)
    {
        if(cylinder_servo[1] == 0){
            cylinder2_move = 1;
        }
    }
    else if (cylinder_servo[1] && cylinder2_move == 1)
    {
        if(cylinder_servo[1] == 0){
            cylinder2_move = 0;
        }
    }

    //回収機構のサーボモーターの制御
    if (cylinder_servo[2] && servo1_angle == 0)
    {
        if(cylinder_servo[2] == 0){
            servo1_angle = 1;
        }
    }
    else if (cylinder_servo[2] && servo1_angle == 1)
    {
        if(cylinder_servo[2] == 0){
            servo1_angle = 0;
        }
    }
}
