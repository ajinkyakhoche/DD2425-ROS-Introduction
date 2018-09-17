#include "ros/ros.h"
#include "ras_lab1_msgs/PWM.h"
#include "ras_lab1_msgs/Encoders.h"
#include "ras_lab1_msgs/ADConverter.h"
#include "geometry_msgs/Twist.h"


class wall_following_controller
{
    //  ros variables
    //  ------------
    // create a node handler object
    ros::NodeHandle n;
    ros::Subscriber adc_sub;
    ros::Publisher target_twist_pub;

    // formulate the target twist message to be published
    geometry_msgs::Twist target_twist;
       
    //  Design variables
    //  -----------------   
    // set desired linear velocity
    float linear_vel = 1;
    // set tuning parameter for wall follower controller
    float gamma = 0.01;

    public:
    // constructor
    wall_following_controller();

    // callback for adc message
    void callback_adc(const ras_lab1_msgs::ADConverter::ConstPtr &);
};

wall_following_controller::wall_following_controller()
{
    ROS_INFO("ENTERED WALL FOLLOWING CONSTRUCTOR");
    
    target_twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);

    // initialize a subscriber to take adc data
    adc_sub = n.subscribe("/kobuki/adc", 1000, &wall_following_controller::callback_adc ,this);
}

void wall_following_controller::callback_adc(const ras_lab1_msgs::ADConverter::ConstPtr& adc_msg)
{
    ROS_INFO("ENTERED ADC CALLBACK");
    // float linear_vel = 1;
    // float gamma = 0.01;
    // geometry_msgs::Twist target_twist;

    float angular_vel = gamma * (adc_msg->ch1 - adc_msg->ch2);
    target_twist.linear.x = linear_vel;
    target_twist.angular.z = angular_vel;
    target_twist_pub.publish(target_twist);
}

int main(int argc, char **argv)
{
    // initialize a ros node
    ros::init(argc, argv, "wall_following_controller");
    
    // initalize object of type wall_following_controller
    wall_following_controller wall_controller;
    ros::spin();

    return 0;
}