// tf_publisher.cpp
#include"tf_publisher.h"

TfPublisher::TfPublisher()
{
}

void TfPublisher::publish(nav_msgs::Odometry odom)
{
	geometry_msgs::TransformStamped  tf_trans;

	tf_trans.header.stamp = odom.header.stamp;
	tf_trans.header.frame_id = odom.header.frame_id;
	tf_trans.child_frame_id = odom.child_frame_id;
	tf_trans.transform.translation.x = odom.pose.pose.position.x;
	tf_trans.transform.translation.y = odom.pose.pose.position.y;
	tf_trans.transform.translation.z = odom.pose.pose.position.z;
	tf_trans.transform.rotation = odom.pose.pose.orientation;

	tf_caster.sendTransform(tf_trans);
}
