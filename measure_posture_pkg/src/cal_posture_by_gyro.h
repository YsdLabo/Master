#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64MultiArray.h>

#ifndef _INCLUDED_CAL_POSTURE_BY_GYRO_H_
#define _INCLUDED_CAL_POSTURE_BY_GYRO_H_

class CalPostureByGyro
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher pub_rpy;
	sensor_msgs::Imu imu_raw;
	std_msgs::Float64MultiArray rpy;
	
	double roll, pitch, yaw;
	double wx_i, wy_i, wz_i;    // オフセット補正用
	
	ros::Time last_stamp;
	bool first_time;
	
public:
	CalPostureByGyro();
	void init();
	void run();
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
};

#endif
