// odom_publisher.h

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#ifndef  _ODOM_PUBLISHER_H_
#define  _ODOM_PUBLISHER_H_

class OdomPublisher
{
private:
	ros::NodeHandle nh;
	ros::Publisher  pub;
public:
	OdomPublisher();
	publish(nav_msgs::Odometry odom);
};

#endif
