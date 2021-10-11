/**
射出機構：エアシリンダー1
回収機構：エアシリンダー2＆サーボモーター1
花火点火機構：サーボモーター2＆LED
足回り：モータ－1＆モーター2＆モーター3
花火点火機構のプログラム
サーボモーターとLEDの制御部分を書いた。
**/

/**********************************************************************
   Include Libraries
**********************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <string>
#include <cstdio>
#include <cmath>

/**********************************************************************
   Declare variables
**********************************************************************/
#define CTRL_PERIOD 0.02f
std_msgs::Float32MultiArray firework_msg;//mbed側に送る
int joy_catch[2];//今のボタンの状態
int last_joy_catch[2];//直前のボタンの状態
float Time = 0;//時間制御の変数
bool servo_mech = false;//サーボモーター制御の状態
bool LED_mech = false;//LED制御の状態
int servo_button_num = 0;
int LED_button_num = 1;
/**********************************************************************
   Proto_type_Declare functions
**********************************************************************/
inline void joy_msg_Callback( const std_msgs::Int32MultiArray::ConstPtr &joy_msg );
inline void Servo();
inline void LED();
/**********************************************************************
   Main
**********************************************************************/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "rob2_firework");
   ros::NodeHandle nh;
   ros::Publisher pub_firework = nh.advertise<std_msgs::Float32MultiArray>("firework", 100);
   ros::Subscriber sub_joy = nh.subscribe("joy_firework", 100, joy_msg_Callback);
   ros::Rate loop_rate(1.0f / CTRL_PERIOD);

   firework_msg.data.resize(2);//{servo2,LED}

   while (ros::ok()) 
   {
      ros::spinOnce();
      
      if(last_joy_catch[servo_button_num] == 1 && joy_catch[servo_button_num] == 0){
         servo_mech = true;
      }
      if(last_joy_catch[LED_button_num] == 1 && joy_catch[LED_button_num] == 0){
         LED_mech = !LED_mech;
      }

      Servo();
      LED();

      pub_firework.publish(firework_msg);  
      
      for (int i = 0; i < 2; i++)
      {
         last_joy_catch[i] = joy_catch[i];
      }
      loop_rate.sleep();
   }
   return 0;
}

/**********************************************************************
   Functions
**********************************************************************/

inline void joy_msg_Callback(const std_msgs::Int32MultiArray::ConstPtr &joy_msg)//joyコンの値を受け取る
{
   for (int i = 0; i < 2; i++)
   {
      joy_catch[i] = joy_msg->data[i];
   }
}

inline void Servo(){//サーボモータ制御
   if(servo_mech == true && Time < 10){
      Time += CTRL_PERIOD;
      firework_msg.data[0] = 0.0;
   }
   else{
      servo_mech = false;
      Time = 0;
      firework_msg.data[0] = 30.0;
   }
}

inline void LED(){//LED制御
   if(LED_mech == true){
      firework_msg.data[1] = 1.0;
   }
   else{
      firework_msg.data[1] = 0.0;
   }
}
