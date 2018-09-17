#include "ros/ros.h"
#include "ras_lab1_msgs/PWM.h"

int main(int argc, char **argv)
{
    // initialize a ros node 
    ros::init(argc, argv, "open_loop_controller");
    ros::NodeHandle n;
    
    // initialize a publisher of type ras_lab1_msgs::PWM
    ros::Publisher pwm_pub = n.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1000);

    // control frequency : 10 Hz
    ros::Rate loop_rate(10);    
    
    while (ros::ok())
    {
        // create an object of class ras_lab1_msgs::PWM
        ras_lab1_msgs::PWM pwm_msg;

        pwm_msg.PWM1 = 150;
        pwm_msg.PWM2 = 100;

        // print info on console
        ROS_INFO("The PWM signals are %d and %d", pwm_msg.PWM1, pwm_msg.PWM2);

        // publish message
        pwm_pub.publish(pwm_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}