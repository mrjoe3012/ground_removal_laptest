#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ground_removal.h"
#include <thread>
#include <mutex>
#include <iostream>
#include <csignal>
#include <X11/Xlib.h>

std::mutex pcMutex;

pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud1(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud2(new pcl::PointCloud<pcl::PointXYZ>());

bool pcAltered = false;
bool useRaw = false;

const unsigned int NUM_BINS = 128, NUM_SEGMENTS = 256;


void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
	// convert to pcl point cloud and update global points
	pcl::PCLPointCloud2 pclpc2;
	pcl_conversions::toPCL(*msg, pclpc2);
	// these pointers are consumed by other threads so we use a mutex
	pcMutex.lock();
	pcl::fromPCLPointCloud2(pclpc2, *pCloud1);
	std::unique_ptr<ground_removal::SegmentArray<pcl::PointXYZ>> segmentArray = ground_removal::assignPointsToBinsAndSegments(*pCloud1, NUM_SEGMENTS, NUM_BINS);
	pCloud2 = ground_removal::groundRemoval<pcl::PointXYZ>(*segmentArray);
	pcAltered = true;
	pcMutex.unlock();
}


// render raw footage
void viewerLoop()
{

	pcl::visualization::CloudViewer viewer(useRaw ? "Raw Playback" : "Processed Playback");
	pcl::visualization::CloudViewer v("asa");

	while(!viewer.wasStopped())
	{
		if(pcAltered && pcMutex.try_lock())
		{
			pcAltered = false;
			if(useRaw)
				viewer.showCloud(pCloud1);
			else
				viewer.showCloud(pCloud2);
			pcMutex.unlock();
		}
	}

	// send signal to ros to kill
	std::raise(SIGINT);

}

int main(int argc, char* argv[])
{
	XInitThreads();

	std::srand(std::time(0));
	ros::init(argc, argv, "laptest" + std::to_string(std::rand()%9999));

	ros::NodeHandle node("~");

	node.getParam("raw", useRaw);

	ros::Subscriber subscriber = node.subscribe("/velodyne_points", 1000, callback);

	std::thread viewerThread(viewerLoop);

	ros::spin();

	return 0;
}
