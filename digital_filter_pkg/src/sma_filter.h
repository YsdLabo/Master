#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#ifndef _INCLUDED_SMA_H_
#define _INCLUDED_SMA_H_

class SMA
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher pub_sma;
	// Messages
	sensor_msgs::Imu  imu_raw;
	sensor_msgs::Imu  imu_sma;
	
	// 移動平均の計算に使用する変数
	int NUM;
	double *buf;
	int num;
	double sma_z;
	
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
	
public:
	SMA() :SMA(1) {};
	SMA(int n);
	~SMA();
	void init(int n);
	void run();
};
#endif
