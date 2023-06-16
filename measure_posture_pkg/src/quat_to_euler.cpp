#include<tf/transform_broadcaster.h>
#include"quat_to_euler.h"

QuatToEuler::QuatToEuler()
{
	init();
}

void QuatToEuler::init()
{
	rpy.data.resize(3);
}

void QuatToEuler::run()
{
	sub_imu = nh.subscribe("/imu/data", 1, &QuatToEuler::imu_callback, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_qte", 1);
}

void QuatToEuler::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;
	
	// クォータニオンを作成
	tf::Quaternion quat;
	quaternionMsgToTF(imu_raw.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	publication();
}

void QuatToEuler::publication()
{
	rpy.data[0] = roll * 180.0 / M_PI;
	rpy.data[1] = pitch * 180.0 / M_PI;
	rpy.data[2] = yaw * 180.0 / M_PI;
	pub_rpy.publish(rpy);
}
