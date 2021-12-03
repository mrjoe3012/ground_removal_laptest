#include "laptest.h"
#include "ground_removal.h"

unsigned int numBins = 128, numSegments = 256;

void callback(const boost::shared_ptr<sensor_msgs::PointCloud2>& msg)
{
	pcl::PCLPointCloud2 pc2;
	pcl_conversions::toPCL(*msg, pc2);

	cloudMutex.lock();
	pcl::fromPCLPointCloud2(pc2, *pCloud);
	std::unique_ptr<ground_removal::SegmentArray<pcl::PointXYZ>> segmentArray = ground_removal::assignPointsToBinsAndSegments<pcl::PointXYZ>(*pCloud, numSegments, numBins);
	pCloud = ground_removal::groundRemoval<pcl::PointXYZ>(*segmentArray);
	renderCloud = true;
	cloudMutex.unlock();

}

int main(int argc, char* argv[])
{

	ros::init(argc, argv, "laptest_play_processed");

	ros::NodeHandle node("~");

	int b = 0, s = 0;
	node.getParam("numBins", b);
	node.getParam("numSegments", s);

	numBins = b > 0 ? static_cast<unsigned int>(b) : numBins;
	numSegments = s > 0 ? static_cast<unsigned int>(s) : numSegments;

	ros::Subscriber subscriber = node.subscribe("/velodyne_points", 1000, callback);

	std::thread viewerThread(viewerLoop, "Processed Playback");

	ros::spin();

	return 0;
}
