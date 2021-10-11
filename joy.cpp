/**********************************************************************
部内ロボコン　機体２　ジョイコン
**********************************************************************/
/**********************************************************************
   Include Libraries
**********************************************************************/
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32MultiArray.h"


/**********************************************************************
   Declare variables(変数宣言)
**********************************************************************/
#define CYCLE_PERIOD 0.02f

//ps4コントローラーの番号を追記
enum MacroesJoyButtons
{ //デジタル値(int)
    BUTTONS_CROSS = 0,
    BUTTONS_CIRCLE = 1,
    BUTTONS_TRIANGLE = 2,
    BUTTONS_SQUARE = 3,
    BUTTONS_L1 = 4,
    BUTTONS_R1 = 5,
    BUTTONS_L2 = 6,
    BUTTONS_R2 = 7,
    BUTTONS_SHARE = 8,
    BUTTONS_OPTIONS = 9,
    BUTTONS_PAIRING = 10,
    BUTTONS_STICK_LEFT = 11,
    BUTTONS_STICK_RIGHT = 12
};
enum MacroesJoyAxes
{ // アナログ値(float)
    AXES_STICK_LEFT_X = 0,
    AXES_STICK_LEFT_Y = 1,
    AXES_BUTTON_L2 = 2,
    AXES_STICK_RIGHT_X = 3,
    AXES_STICK_RIGHT_Y = 4,
    AXES_BUTTON_R2 = 5,
    AXES_BUTTON_CROSS_X = 6,
    AXES_BUTTON_CROSS_Y = 7
};

std::vector<int> JoyButtonsArray(13, 0);
std::vector<float> JoyAxesArray(8, 0.0);

float a = 30;

std_msgs::Float32MultiArray msg_float;  //速度、旋回
std_msgs::Int32MultiArray emission; //射出・回収
std_msgs::Int32MultiArray shot;     //花火

/**********************************************************************
  Proto_type_Declare functions
**********************************************************************/
void joy_ps3_Callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

/**********************************************************************
   Main
**********************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_omuni");
    ros::NodeHandle nh;
    ros::Subscriber sub_first = nh.subscribe("joy", 1, joy_ps3_Callback);                       //joyコンの値を受け取る

    /*足回り用*/
    ros::Publisher pub_counter = nh.advertise<std_msgs::Float32MultiArray>("joy_content", 100); //速度,旋回速度を送る
     /*射出、回収用*/
    ros::Publisher pub_retrieve_launch = nh.advertise<std_msgs::Int32MultiArray>("joy_retrieve_launch", 100); //シリンダー１、シリンダー２、サーボモーター１用送る
    /*花火打ち上げ用*/
    ros::Publisher pub_firework = nh.advertise<std_msgs::Int32MultiArray>("joy_firework", 100);//サーボモーター２,LED

    ros::Rate loop_rate(1.0f / CYCLE_PERIOD);

    while (ros::ok())
    {
        ros::spinOnce(); //callback関数を読み込む

        /*足回り用*/
        msg_float.data.resize(3);
        msg_float.data[0] = a * JoyAxesArray[AXES_STICK_LEFT_X]; //x軸に対する速度
        msg_float.data[1] = a * JoyAxesArray[AXES_STICK_LEFT_Y]; //ｙ軸に対する速度
        msg_float.data[2] = JoyAxesArray[AXES_STICK_RIGHT_X];    //旋回速度

        /*射出、回収用*/
        emission.data.resize(3);
        emission.data[0] = JoyButtonsArray[BUTTONS_CIRCLE]; //シリンダー１
        emission.data[1] = JoyButtonsArray[BUTTONS_CROSS]; //シリンダー２
        emission.data[2] = JoyButtonsArray[BUTTONS_SQUARE];//サーボモーター1        
        
        /*花火打ち上げ用*/
        shot.data.resize(2);
        shot.data[0] = JoyButtonsArray[BUTTONS_L2]; //サーボモーター２
        shot.data[1] = JoyButtonsArray[BUTTONS_R2]; //LED


        pub_counter.publish(msg_float);
        pub_retrieve_launch.publish(emission);
        pub_firework.publish(shot);

        loop_rate.sleep();
    }
    return 0;
}

/**********************************************************************
   Functions <Call Back>
**********************************************************************/

void joy_ps3_Callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    JoyButtonsArray = joy_msg->buttons;
    JoyAxesArray = joy_msg->axes;
}
