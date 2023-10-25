#include "riemann_sum.h"

RiemannSum::RiemannSum()
{
	init();
}

void RiemannSum::init()
{
	first_time = true;
}

void RiemannSum::run()
{
	sub_imu = nh.subscribe("/imu", 1, &RiemannSum::imu_callback, this);
	pub_angle = nh.advertise<std_msgs::Float64>("riemann_angle_z", 1);
}

void RiemannSum::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	if( first_time == false )
	{
		// 区分積分
		【追記】
		
		publication();
	}
	else
	{
		angle_z = 0.0;
		first_time = false;
	}
	last_stamp = imu_raw.header.stamp;
}

void RiemannSum::publication()
{
	std_msgs::Float64 angle;
	angle.data = angle_z;
	pub_angle.publish(angle);
	ROS_INFO("angle(riemann) : %lf", angle.data);
}



