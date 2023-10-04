#include "rc_filter.h"

RCFilter::RCFilter(double fc) :Fc(fc)
{
	init();
}

void RCFilter::init()
{
	first_time = true;
}

void RCFilter::run()
{
	sub_imu = nh.subscribe("/imu", 1, &RCFilter::imu_callback, this);    // /imuを購読
	pub_rcf = nh.advertise<sensor_msgs::Imu>("/imu_rcf", 1);             // /imu_rcfに配信
}

void RCFilter::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	double acc_z = imu_raw.linear_acceleration.z;
	
	if( !first_time )
	{
		dt = (imu_raw.header.stamp - last_stamp).toSec();
		// アルファの計算
		alpha = 【追記】;
		// フィルタの計算
		rcf_z = 【追記】;
		
		publication();
	}
	else
	{
		rcf_z = acc_z;
		first_time = false;
	}
	last_stamp = imu_raw.header.stamp;
}

void RCFilter::publication()
{
	imu_rcf.header.stamp = imu_raw.header.stamp;
	imu_rcf.linear_acceleration.z = rcf_z;
	pub_rcf.publish(imu_rcf);
	ROS_INFO("RCF: %lf", rcf_z);
}
