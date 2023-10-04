#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#ifndef _INCLUDED_RC_H_
#define _INCLUDED_RC_H_

class RCFilter
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_imu;
	ros::Publisher pub_rcf;
	// Messages
	sensor_msgs::Imu  imu_raw;
	sensor_msgs::Imu  imu_rcf;
	
	// フィルタに必要な変数
	double Fc;
	double rcf_z;
	double alpha;
	double dt;
	ros::Time last_stamp;
	bool first_time;
	
	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void publication();
	
public:
	RCFilter() = delete;
	RCFilter(double fc);
	void init();
	void run();
};
#endif
