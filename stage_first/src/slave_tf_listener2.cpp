

// Slave follower implementation

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include "ros/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

double intensities[27];
double mul = 1;

std::string flag = "line";
geometry_msgs::Twist m_vel;
;

ros::Publisher slave_vel;

// Function declerations to avoid, rotate and move
void avoid(void);
void rotate(double angular_speed, double relative_angle, bool clockwise);

// Laser call back function decleration
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
void flagCallBack(const std_msgs::String::ConstPtr &msg);
void velCallBack(const geometry_msgs::Twist::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    slave_vel = node.advertise<geometry_msgs::Twist>("/ares2/cmd_vel", 10);

    ros::Subscriber laser = node.subscribe("/ares2/scan", 1, laserCallBack);
    ros::Subscriber sub_flag = node.subscribe("/flag", 1, flagCallBack);
    ros::Subscriber sub_vel = node.subscribe("cmd_vel", 1, velCallBack);

    tf::TransformListener listener;

    ros::Rate rate(100.0);
    while (node.ok())
    {
        tf::StampedTransform transformSM;
        tf::StampedTransform transformMS;
        // From Slave to master transformation
        try
        {
            listener.lookupTransform("/ares2", "/ares1", ros::Time(0), transformSM);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // From Master to Slave transformation
        try
        {
            listener.lookupTransform("/ares1", "/ares2", ros::Time(0), transformMS);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        avoid();  // function call to avoid obstacles while following the Master

        // Proportional controller to follow the Master (Go to master behaviour)

        float x = transformSM.getOrigin().x();
        float y = transformSM.getOrigin().y();
        float theta = tf::getYaw(transformSM.getRotation());
        if (flag == std::string("line"))
        {
            // ROS_INFO("robot2 is in line");
            float dx = -0.5, dy = 0;
            x += dx * std::cos(theta) - dx * std::sin(theta);
        }
        else if (flag == std::string("triangle"))
        {
            float dx = -0.5, dy = 0.5;
            x += dx * std::cos(theta) - dx * std::sin(theta);
            y += dy * std::sin(theta) + dy * std::cos(theta);
            // ROS_INFO("robot2 is in triangle");
        }

        geometry_msgs::Twist vel_msg;
        if (m_vel.linear.x > 1e-3)
        {
            vel_msg.angular.z = 4.0 * atan2(y, x);
            // vel_msg.linear.x = 3 * sqrt(pow(x, 2) + pow(y, 2));
            vel_msg.linear.x = (pow(x, 2) + pow(y, 2));
        }

        slave_vel.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};

void flagCallBack(const std_msgs::String::ConstPtr &msg)
{
    flag = msg->data.c_str();
    ROS_INFO("robot 2 is in %s", flag.c_str());
}

/*
 * Call back implementation to read and process laser data
 */
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
    for (int i = 0; i < 27; i++)  // I need not loop to copy, I not familiar with std::vectors
    {
        intensities[i] = laser_msg->intensities[i];
        mul = mul * intensities[i];  // check point if robot is blocked 270 degrees
    }
}

void velCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    m_vel = *msg;
}

/*
 * Obstacle avoidance behaviour implementation
 */
void avoid(void)
{
    // ROS_INFO("I am [%s]", "avoiding");

    int samples = 27;
    int fov = 4.7123;
    double inc = 0.1745;  // 270/27 degrees to radians
    int center = samples / 2;
    /*if (mul == 1)// blocked around 270 degrees
    {
    rotate(1.0, 3.1415, 1); //about turn
    }*/
    if ((intensities[center - 1] == 1) || (intensities[center] == 1) ||
        (intensities[center + 1] == 1))  // obstacle in front
    {
        // Check one by one on both sides of the robot to determine free space and rotate by the amount scanned in a
        // first free direction
        for (int i = 2; i < center; i++)
        {
            if (intensities[center - i] == 0)  // no obstacle
            {
                rotate(1.0, (i + 1) * inc, 1);
                break;
            }
            else if (intensities[center + i] == 0)  // no obstacle
            {
                rotate(1.0, (i + 1) * inc, 0);
                break;
            }
        }
    }
    else
    {
        // move(1.0, 1.0, 1);
    }
}

/**
 *  makes the robot turn with a certain angular velocity,
 *  either clockwise or counter-clockwise direction
 */
void rotate(double angular_speed, double relative_angle, bool clockwise)
{
    // angular_speed = degrees2radians(angular_speed);
    // relative_angle = degrees2radians(relative_angle);
    geometry_msgs::Twist vel_msg;
    // set a random linear velocity in the x-axis
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // set a random angular velocity in the y-axis
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    // Control strategy for clockwise and counter-clockwise directions
    if (clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;
    ros::Rate loop_rate(1000);
    // Condition to teminate rotation after rotating to the required orientation
    do
    {
        slave_vel.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();

    } while (current_angle < relative_angle);
    vel_msg.angular.z = 0;
    slave_vel.publish(vel_msg);
}
