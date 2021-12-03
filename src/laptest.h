#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <thread>
#include <mutex>
#include <csignal>
#include <iostream>

std::mutex cloudMutex;
pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);

bool renderCloud = false;

void viewerLoop(std::string viewerName = "Viewer")
{
	pcl::visualization::CloudViewer viewer(viewerName);
	
	while(!viewer.wasStopped())
	{
		if(renderCloud && cloudMutex.try_lock())
		{
			renderCloud = false;
			viewer.showCloud(pCloud);
			cloudMutex.unlock();
		}
	}

	std::raise(SIGINT);
}
