#include<ros/ros.h>
#include<tf/transform_broadcaster.h>

#ifndef  _TF_PUBLISHER_H_
#define  _TF_PUBLISHER_H_

class TfPublisher
{
private:
	ros::NodeHandle nh;
	tf::TransformBroadcaster tf_caster;
public:
	TfPublisher();
	publish(nav_msgs::Odometry odm);
};

#endif
