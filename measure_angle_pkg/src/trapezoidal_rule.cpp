#include "trapezoidal_rule.h"

TrapezoidalRule::TrapezoidalRule()
{
	init();
}

void TrapezoidalRule::init()
{
	first_time = true;
}

void TrapezoidalRule::run()
{
	sub_imu = nh.subscribe("/imu", 1, &TrapezoidalRule::imu_callback, this);
	pub_angle = nh.advertise<std_msgs::Float64>("trapezoidal_angle_z", 1);
}

void TrapezoidalRule::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	if( first_time == false )
	{
		// 台形区分求積法
		【追記】
		
		publication();
	}
	else
	{
		angle_z = 0.0;
		first_time = false;
	}
	last_stamp = imu_raw.header.stamp;
	last_angular_z = (double)imu_raw.angular_velocity.z;
}

void TrapezoidalRule::publication()
{
	std_msgs::Float64 angle;
	angle.data = angle_z;
	pub_angle.publish(angle);
	ROS_INFO("angle(trapezoidal) : %lf", angle.data);
}



