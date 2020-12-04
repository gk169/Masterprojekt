#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include <thread>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <math.h>

#include "../include/run_basics/organize_velodyne_cloud.h"

//pcl::io::savePCDFileASCII ("/home/micha/Masterprojekt/cropped_cloud.pcd", *dataOut.get());
//std::cout << "---------------" << std::endl;
//std::this_thread::sleep_for(std::chrono::seconds(3));

//typedef pcl::PointXYZ PointType;
#define PI 3.14159265

ros::Subscriber subVeloFront;
ros::Subscriber subVeloTop;
ros::Subscriber subVeloBack;
ros::Publisher croppedCloud_pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr FrontCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr TopCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr BackCloud;
std_msgs::Header header;

bool getInAngle(pcl::PointXYZI point, int angleStart, int angleEnd)
{
     float anglePoint = atan2(-point.y, point.x) * 180 / PI;
    //front: 134,5 / -135,5
    //back: 135 / -135
	
    //check angle
	if (anglePoint > angleStart && anglePoint < angleEnd)
        return true;

	return false;
}

pcl::PointXYZI transformFrontToTopVelo(pcl::PointXYZI point)
{
	float sX = 1.25; //Distance Top to Front!
	float sY = 0.66;
	float sZ = -1.83;

	float shiftAngle = 45.5;
	//float pX = (sqrt(2)/2)*point.x-(sqrt(2)/2)*point.y; //Position of Point, rotated 45°!
	//float pY = (sqrt(2)/2)*point.x+(sqrt(2)/2)*point.y;
	float pX = (cos(shiftAngle * PI / 180.0))*point.x-(sin(shiftAngle * PI / 180.0))*point.y; //Position of Point, rotated 45°!
	float pY = (sin(shiftAngle * PI / 180.0))*point.x+(cos(shiftAngle * PI / 180.0))*point.y;
	float pZ = point.z;

	pcl::PointXYZI ret_point;

	ret_point.x = sX + pX;
	ret_point.y = sY + pY;
	ret_point.z = sZ + pZ;
	//ret_point.ring = point.ring;
	ret_point.intensity = point.intensity;

	return ret_point;
}

pcl::PointXYZI transformBackToTopVelo(pcl::PointXYZI point)
{
	float sX = -1.11; //Distance Top to Back!
	float sY = -0.66;
	float sZ = -1.78;

	float shiftAngle = -135;
	//float pX = (sqrt(2)/2)*point.x-(sqrt(2)/2)*point.y; //Position of Point, rotated 135°!
	//float pY = (sqrt(2)/2)*point.x+(sqrt(2)/2)*point.y;
	float pX = (cos(shiftAngle * PI / 180.0))*point.x-(sin(shiftAngle * PI / 180.0))*point.y; //Position of Point, rotated 135°!
	float pY = (sin(shiftAngle * PI / 180.0))*point.x+(cos(shiftAngle * PI / 180.0))*point.y;
	float pZ = point.z;

	pcl::PointXYZI ret_point;

	ret_point.x = sX + pX;
	ret_point.y = sY + pY;
	ret_point.z = sZ + pZ;
	//ret_point.ring = point.ring;
	ret_point.intensity = point.intensity;

	return ret_point;
}

void process_topics()
{
	if(nullptr == FrontCloud || nullptr == TopCloud || nullptr == BackCloud) return;

	ROS_INFO("PcFusion - All topics received, start processing!");

	pcl::PointCloud<pcl::PointXYZI>::Ptr dataOut( new pcl::PointCloud<pcl::PointXYZI> );

	//pcl::copyPointCloud<pcl::PointXYZI>(*TopCloud, *dataOut);

	pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud( new pcl::PointCloud<pcl::PointXYZI> );

	//add top points to pointcloud
	pcl::copyPointCloud<pcl::PointXYZI>(*TopCloud, *fullCloud);

	//add front points to pointcloud
	for (std::size_t i = 0; i < FrontCloud->points.size (); ++i)
	{
            if (getInAngle(FrontCloud->points[i], -120, 120))
            {
        		fullCloud->points.push_back(transformFrontToTopVelo(FrontCloud->points[i]));
        		fullCloud->width++;// = fullCloud->width+1;
            }
	}
	//add back points to pointcloud
	for (std::size_t i = 0; i < BackCloud->points.size (); ++i)
	{
            if (getInAngle(BackCloud->points[i], -115, 115))
            {
                fullCloud->points.push_back(transformBackToTopVelo(BackCloud->points[i]));
                fullCloud->width = fullCloud->width+1;
            }
	}

	/* Use fullcloud instead only FOW */
	dataOut = fullCloud;

	sensor_msgs::PointCloud2 out_msg;
	pcl::toROSMsg(*dataOut.get(),out_msg );

	out_msg.header=header;
	ROS_INFO("PcFusion - Publishing preprocessed-pointCloud!");
	//ROS_INFO("PcFusion - Header: %d - %d - %d ",out_msg.header.seq, out_msg.header.stamp.sec, out_msg.header.stamp.nsec);

	FrontCloud = nullptr;
	TopCloud = nullptr;
	BackCloud = nullptr;
	croppedCloud_pub.publish(out_msg);
}

void veloFrontCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	ROS_INFO("PcFusion - Recived new Front-pointCloud!");

	header = data->header;

	//convert ROS::PointCloud2 to PCL::PointCloud2
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*data, pcl_pc2);
	//convert PCL::PointCloud2 to PCL::PointCloud<pcl::PointXYZI>
	FrontCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	fromPCLPointCloud2(pcl_pc2, *FrontCloud);

	process_topics();
}

void veloTopCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	ROS_INFO("PcFusion - Recived new Top-pointCloud!");

	//convert ROS::PointCloud2 to PCL::PointCloud2
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*data, pcl_pc2);
	//convert PCL::PointCloud2 to PCL::PointCloud<pcl::PointXYZI>
	TopCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	fromPCLPointCloud2(pcl_pc2, *TopCloud);

	process_topics();
}

void veloBackCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	ROS_INFO("PcFusion - Recived new Back-pointCloud!");

	//convert ROS::PointCloud2 to PCL::PointCloud2
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*data, pcl_pc2);
	//convert PCL::PointCloud2 to PCL::PointCloud<pcl::PointXYZI>
	BackCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	fromPCLPointCloud2(pcl_pc2, *BackCloud);

	process_topics();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PcFusion");

	ros::NodeHandle nh;

	subVeloFront = nh.subscribe("/BugaSegm/pc_segm_front", 1000, veloFrontCallback);
	subVeloTop = nh.subscribe("/BugaSegm/pc_segm_top", 1000, veloTopCallback);
	subVeloBack = nh.subscribe("/BugaSegm/pc_segm_back", 1000, veloBackCallback);
	croppedCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/pc_segm", 1000);

	ros::spin();

	return 0;
}
