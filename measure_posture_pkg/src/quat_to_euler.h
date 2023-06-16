#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64MultiArray.h>

#ifndef _INCLUDED_QUAT_TO_EULER_H_
#define _INCLUDED_QUAT_TO_EULER_H_

class QuatToEuler
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher  pub_rpy;
	sensor_msgs::Imu imu_raw;
	std_msgs::Float64MultiArray rpy;
	
	double roll, pitch, yaw;
public:
	QuatToEuler();
	void init();
	void run();
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
};

#endif
