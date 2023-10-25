#include "cal_posture_by_gyro.h"

CalPostureByGyro::CalPostureByGyro()
{
	init();
}

void CalPostureByGyro::init()
{
	rpy.data.resize(3);
	wx_i = wy_i = wz_i = 0.0;
	roll = pitch = yaw = 0.0;
	first_time = true;
}

void CalPostureByGyro::run()
{
	sub_imu = nh.subscribe("/imu", 1, &CalPostureByGyro::imu_callback, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_gyro", 1);
}

void CalPostureByGyro::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	static int count = 0;

	imu_raw = *msg;
	
	double wx = imu_raw.angular_velocity.x;
	double wy = imu_raw.angular_velocity.y;
	double wz = imu_raw.angular_velocity.z;
	
	if(!first_time) {
		// オフセット補正
		wx = wx - wx_i;
		wy = wy - wy_i;
		wz = wz - wz_i;
		
		double dt = 【追記】;
		
		double dx = 【追記】;
		double dy = 【追記】;
		double dz = 【追記】;
		double Sr = 【追記】;
		double Cr = 【追記】;
		double Tp = 【追記】;
		double Cp = 【追記】;
		
		roll += 【追記】;
		pitch += 【追記】;
		yaw += 【追記】;
	
		publication();
	}
	else {
		wx_i += wx;
		wy_i += wy;
		wz_i += wz;
		count ++;
		if(count == 10) {
			wx_i /= 10.0;
			wy_i /= 10.0;
			wz_i /= 10.0;
			first_time = false;
		}
	}
	last_stamp = imu_raw.header.stamp;
}

void CalPostureByGyro::publication()
{
	rpy.data[0] = roll * 180.0 / M_PI;
	rpy.data[1] = pitch * 180.0 / M_PI;
	rpy.data[2] = yaw * 180.0 / M_PI;
	pub_rpy.publish(rpy);
}

