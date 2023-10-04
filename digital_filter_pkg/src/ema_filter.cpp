#include "ema_filter.h"

EMA::EMA(double a) :alpha(a) 
{
	init();
}

void EMA::init()
{
	first_time = true;
}

void EMA::run()
{
	sub_imu = nh.subscribe("/imu", 1, &EMA::imu_callback, this);    // /imuを購読
	pub_ema = nh.advertise<sensor_msgs::Imu>("/imu_ema", 1);       // /imu_emaに配信
}

void EMA::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	double acc_z = imu_raw.linear_acceleration.z;

	if(first_time == false)
	{
		// 指数移動平均の計算
		ema_z = 【追記】;
	
		publication();
	}
	else
	{
		ema_z = acc_z;
		first_time = false;
	}
}

void EMA::publication()
{
	imu_ema.header = imu_raw.header;
	imu_ema.linear_acceleration.z = ema_z;
	pub_ema.publish(imu_ema);
	ROS_INFO("EMA: %lf", ema_z);
}

