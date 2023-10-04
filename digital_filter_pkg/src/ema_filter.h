#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#ifndef _INCLUDED_EMA_H_
#define _INCLUDED_EMA_H_

class EMA
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher pub_ema;
	// Messages
	sensor_msgs::Imu  imu_raw;
	sensor_msgs::Imu  imu_ema;
	
	// 指数移動平均に必要な変数
	double alpha;
	bool first_time;
	double ema_z;
	
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
	
public:
	EMA() = delete;
	EMA(double a);
	void init();
	void run();
};
#endif
