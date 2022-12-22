#include"odom_publisher.h"

OdomPublisher::OdomPublisher()
{
	pub = nh.advertise<nav_msgs::Odometry>("odom", 10, this);
}

void OdomPublisher::publish(nav_msgs::Odometry odom)
{
	pub.publish(odom);
}
