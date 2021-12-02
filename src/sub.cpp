#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "ground_removal.h"
#include <thread>

int main(int argc, char* argv[])
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>());

	ros::init(argc, argv, "listener");
	
	ros::spin();

	return 0;
}
