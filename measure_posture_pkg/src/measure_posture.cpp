#include"cal_posture_by_gravity.h" 
//#include"cal_posture_by_gyro.h" 

int main(int argc, char** argv)
{
	ros::init(argc, argv, "measure_posture");
	
	CalPostureByGravity grav;
	grav.run();
	
	//CalPostureByGyro gyro;
	//gyro.run();
	
	ros::spin();
	return 0;
}
