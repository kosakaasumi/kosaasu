/**
射出機構：エアシリンダー1
回収機構：エアシリンダー2＆サーボモーター1
花火点火機構：サーボモーター2＆LED
足回り：モータ－1＆モーター2＆モーター3
mbed側のプログラム。buildはエラー無く通ったけど、動くかどうかは未知数。ポート番号は要変更。
**/
/**********************************************************************
Include Libraries
**********************************************************************/
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "servo_motor.h"
#include "dc_motor_1.h"
/**********************************************************************
Declare MACRO
**********************************************************************/
#define CTRL_PERIOD 0.02f
/**********************************************************************
Proto_type_Declare functions
**********************************************************************/
inline void firework_Callback(const std_msgs::Float32MultiArray &msg);
inline void SetOrder();
inline void retrieve_launch_cb(const std_msgs::Float32MultiArray &msg); //回収機構と発射機構のcb
/**********************************************************************
Declare variables
**********************************************************************/
Timer ControlTicker;
float order_catch[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; //モーター1、モーター2、モーター3、エアシリンダー1、エアシリンダー2、サーボモーター1、サーボモーター2、LEDの値を格納
/**********************************************************************
Declare Instances
**********************************************************************/
Serial PC(USBTX, USBRX, 115200);
dc_motor_1 Motor1(D3, D8, 0);
dc_motor_1 Motor2(D3, D8, 0);
dc_motor_1 Motor3(D3, D8, 0);
DigitalOut cylinder1(A1);
DigitalOut cylinder2(A1);
ServoMotor servo1(D4, 0.0);
ServoMotor servo2(D4, 0.0);
DigitalOut LED(A1);

ros::Subscriber<std_msgs::Float32MultiArray> order_sub("firework", &firework_Callback);
ros::Subscriber<std_msgs::Float32MultiArray> sub_retrieve_launch("robo_retrieve_launch", &retrieve_launch_cb);
int main(int argc, char **argv)
{
  ControlTicker.start();
  ros::NodeHandle n;
  n.getHardware()->setBaud(115200);
  n.initNode();
  n.subscribe(order_sub);

  for (;;)
  {
    if (ControlTicker.read() >= CTRL_PERIOD)
    {
      ControlTicker.reset();
      SetOrder();
      n.spinOnce();
    }
  }
}

inline void SetOrder()
{
  Motor1.drive(order_catch[0]);
  Motor2.drive(order_catch[1]);
  Motor3.drive(order_catch[2]);
  cylinder1 = order_catch[3];
  cylinder2 = order_catch[4];
  servo1.rot(order_catch[5]);
  servo2.rot(order_catch[6]);
  LED = order_catch[7];
}

inline void firework_Callback(const std_msgs::Float32MultiArray &msg) //
{
  order_catch[6] = msg.data[0];
  order_catch[7] = msg.data[1];
}

inline void retrieve_launch_cb(const std_msgs::Float32MultiArray &msg)
{
  order_catch[3] = msg.data[0]; //シリンダー1（発射機構）
  order_catch[4] = msg.data[1]; //シリンダー2（回収機構）
  order_catch[5] = msg.data[2]; //サーボモーター1（回収機構）
}
