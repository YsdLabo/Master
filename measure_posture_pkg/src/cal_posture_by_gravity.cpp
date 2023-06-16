#include "cal_posture_by_gravity.h"

CalPostureByGravity::CalPostureByGravity()
{
	init();
}

void CalPostureByGravity::init()
{
	rpy.data.resize(3);
	first_time = true;
}

void CalPostureByGravity::run()
{
	sub_imu = nh.subscribe("/imu_rcf", 1, &CalPostureByGravity::imu_callback, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_grav", 1);
}

void CalPostureByGravity::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	if(!first_time) {
		double ax = imu_raw.linear_acceleration.x;
		double ay = imu_raw.linear_acceleration.y;
		double az = imu_raw.linear_acceleration.z;
		roll = 【追記】;
		pitch = 【追記】;
		yaw = 0.0;
		publication();
	}
	else {
		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
		first_time = false;
	}
}

void CalPostureByGravity::publication()
{
	rpy.data[0] = roll * 180.0 / M_PI;
	rpy.data[1] = pitch * 180.0 / M_PI;
	rpy.data[2] = yaw * 180.0 / M_PI;
	pub_rpy.publish(rpy);
}

