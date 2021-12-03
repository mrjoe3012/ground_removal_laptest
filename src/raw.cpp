#include "laptest.h"

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
	pcl::PCLPointCloud2 pc2;
	pcl_conversions::toPCL(*msg, pc2);

	cloudMutex.lock();
	pcl::fromPCLPointCloud2(pc2, *pCloud);
	renderCloud = true;
	cloudMutex.unlock();

}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "laptest_play_raw");
	
	ros::NodeHandle node;

	ros::Subscriber subscriber = node.subscribe("/velodyne_points", 1000, callback);

	std::thread viewerThread(viewerLoop, "Raw Playback");

	ros::spin();

	return 0;

}
