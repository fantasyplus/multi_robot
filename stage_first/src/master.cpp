

// Master node should guide the master robot autonomously without bumping into
// objects and the slave should subscribe for master position
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

geometry_msgs::Twist m_vel;

void velCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    m_vel = *msg;
    // ROS_INFO("I heard: [%f]", m_vel.linear.x);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master");
    ros::NodeHandle masterNode;

    ros::Publisher velocity_publisher = masterNode.advertise<geometry_msgs::Twist>("/ares1/cmd_vel", 1000);
    ros::Subscriber sub_vel = masterNode.subscribe("cmd_vel", 1, velCallBack);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg = m_vel;
        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
