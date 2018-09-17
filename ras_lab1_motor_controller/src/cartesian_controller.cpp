#include "ros/ros.h"
#include "ras_lab1_msgs/PWM.h"
#include "ras_lab1_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"

#define PIE 3.14159268

class cartesian_controller
{
    // design variables of robot
    int ticks_per_rev = 360;
    float b = 0.23;
    float r = 0.0352;
    int control_freq = 10;
    float dt = 1.0 / control_freq;

    // variables for PI control
    float error1 = 0.0;
    float int_error1 = 0.0;
    float error2 = 0.0;
    float int_error2 = 0.0;
    float alpha1 = 15.0;
    float alpha2 = 13.2;
    float beta1 = 0.4;
    float beta2 = 0.5;
    float desired_w1 = 0;
    float estimated_w1 = 0;
    float desired_w2 = 0;
    float estimated_w2 = 0;

  public:
    // formulate the PWM message
    ras_lab1_msgs::PWM pwm_msg;

    //  Member functions
    // ------------------
    // callback for desired twist message
    void callback_desired_twist(const geometry_msgs::Twist::ConstPtr &);

    // callback for encoder message
    void callback_observed_twist(const ras_lab1_msgs::Encoders::ConstPtr &);

    // callback for encoder message
    void control_action();

    // function for PI control
    int PI_control(float alpha, float beta, float desired_w, float estimated_w, float error, float int_error);
};

void cartesian_controller::callback_desired_twist(const geometry_msgs::Twist::ConstPtr &twist_msg)
{
    ROS_INFO("The desired twist is %f and %f", twist_msg->linear.x, twist_msg->angular.z);

    //geometry_msgs::Twist twist_desired = twist_msg;
    float v = twist_msg->linear.x;
    float w = twist_msg->angular.z;

    desired_w1 = (v + w * b) / r; // left wheel omega
    desired_w2 = (v - w * b) / r; // right wheel omega
}

void cartesian_controller::callback_observed_twist(const ras_lab1_msgs::Encoders::ConstPtr &encoder_msg)
{
    float delta1 = encoder_msg->delta_encoder1;
    float delta2 = encoder_msg->delta_encoder2;

    estimated_w1 = 2 * PIE * control_freq * delta1 / ticks_per_rev;
    estimated_w2 = 2 * PIE * control_freq * delta2 / ticks_per_rev;
}

void cartesian_controller::control_action()
{
    pwm_msg.PWM1 = PI_control(alpha1, beta1, desired_w1, estimated_w1, error1, int_error1);
    pwm_msg.PWM2 = PI_control(alpha2, beta2, desired_w2, estimated_w2, error2, int_error2);
}

int cartesian_controller::PI_control(float alpha, float beta, float desired_w, float estimated_w, float error, float int_error)
{
    // if(motor_idx == 1)
    // {
    //     float alpha = alpha1;
    //     float beta = beta1;
    //     float desired_w = desired_w1;
    //     float estimated_w = estimated_w1;
    // }
    // else if(motor_idx == 2)
    // {
    //     float alpha = alpha2;
    //     float beta = beta2;
    //     float desired_w = desired_w2;
    //     float estimated_w = estimated_w2;
    // }

    error = desired_w - estimated_w;
    int_error = int_error + error * dt;
    float temp = alpha * error + beta * int_error;
    int pwm = (int)temp;

    return pwm;
}

int main(int argc, char **argv)
{
    // create a node handler object
    ros::NodeHandle n;

    // initialize a ros node
    ros::init(argc, argv, "cartesian_controller");

    // initialize object of class cartesian_controller()
    cartesian_controller controller;

    // initialize a subscriber for taking desired twist (here published manually)
    //ros::Subscriber desired_twist_sub = n.subscribe("/motor_controller/twist", 1000, &cartesian_controller::callback_desired_twist);
    ros::Subscriber desired_twist_sub = n.subscribe("/motor_controller/twist", 1000, &cartesian_controller::callback_desired_twist, &controller);

    // initialize a subscriber for taking desired twist (here published manually)
    ros::Subscriber observed_twist_sub = n.subscribe("/kobuki/encoders", 1000, &cartesian_controller::callback_observed_twist, &controller);

    // initialize a ROS Publisher of type ras_lab1_msgs::PWM
    ros::Publisher pwm_pub = n.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1000);

    // control frequency : 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        controller.control_action();
        pwm_pub.publish(controller.pwm_msg);
        loop_rate.sleep();
    }

    ros::spin();
}