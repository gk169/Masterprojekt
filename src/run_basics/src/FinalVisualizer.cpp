#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
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

#include <run_basics/ObjectList.h>
#include <run_basics/singleObject.h>
#include <run_basics/MinMaxMean.h>

#include "../include/run_basics/organize_velodyne_cloud.h"

ros::Subscriber subCloud;
ros::Subscriber subObjects;

run_basics::ObjectList::ConstPtr fullObjectList=nullptr;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	//ROS_INFO("Recived new pointCloud!");

	float CAM_PosX, CAM_PosY, CAM_PosZ, CAM_OrtX, CAM_OrtY, CAM_OrtZ;
	float PCL_X, PCL_Y, PCL_Z, PCL_Roll, PCL_Yaw, PCL_Pitch;
	ros::param::get("/CAM_PosX", CAM_PosX);
	ros::param::get("/CAM_PosY", CAM_PosY);
	ros::param::get("/CAM_PosZ", CAM_PosZ);
	ros::param::get("/CAM_OrtX", CAM_OrtX);
	ros::param::get("/CAM_OrtY", CAM_OrtY);
	ros::param::get("/CAM_OrtZ", CAM_OrtZ);

	ros::param::get("/PCL_X", PCL_X);
	ros::param::get("/PCL_Y", PCL_Y);
	ros::param::get("/PCL_Z", PCL_Z);
	ros::param::get("/PCL_Roll", PCL_Roll);
	ros::param::get("/PCL_Pitch", PCL_Pitch);
	ros::param::get("/PCL_Yaw", PCL_Yaw);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*data, pcl_pc2);
	fromPCLPointCloud2(pcl_pc2, *cloud);

	// Display pointcloud:
	std::vector<std::vector<int>> bgrVals;
	bgrVals.push_back(std::vector<int>{128, 128, 128});    //Sky
	bgrVals.push_back(std::vector<int>{0, 0, 128});	    //Building	
	bgrVals.push_back(std::vector<int>{128, 192, 192});    //Pfosten
	bgrVals.push_back(std::vector<int>{0, 69, 255});	    //Road-Marking
	bgrVals.push_back(std::vector<int>{128, 64, 128});	    //Road
	bgrVals.push_back(std::vector<int>{222, 40, 60});	    //Pavement
	bgrVals.push_back(std::vector<int>{0, 128, 128});      //Grünzeug
	bgrVals.push_back(std::vector<int>{128, 128, 192});    //Sign-Symbol
	bgrVals.push_back(std::vector<int>{128, 64, 64});	    //Fence
	bgrVals.push_back(std::vector<int>{128, 0, 64});	    //Vehicle
	bgrVals.push_back(std::vector<int>{0, 64, 64});        //Person
	bgrVals.push_back(std::vector<int>{192, 128, 0});	    //Bike
	bgrVals.push_back(std::vector<int>{255, 255, 255});	    //Not matched in image
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::copyPointCloud(*cloud, *rgb_cloud);
	//std::cout << "before for loop" << std::endl;
	for (std::uint64_t i = 0; i < cloud->points.size (); ++i)
	{
		//std::cout << "in for loop " << i << " " << cloud->points[i].intensity << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
		//std::cout << bgrVals[cloud->points[i].intensity][0] << std::endl;
		//std::cout << bgrVals[cloud->points[i].intensity][1] << std::endl;
		//std::cout << bgrVals[cloud->points[i].intensity][2] << std::endl;
		pcl::PointXYZRGB point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		point.z = cloud->points[i].z;
		point.b = bgrVals[cloud->points[i].intensity][0];
		point.g = bgrVals[cloud->points[i].intensity][1];
		point.r = bgrVals[cloud->points[i].intensity][2];
		rgb_cloud->points.push_back(point);
	}
	
	//std::cout << "afteer for loop" << std::endl;

	viewer.removePointCloud("input_cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rgb_cloud);
	//std::cout << "afteer PointCloudColorHandlerRGBField" << std::endl;
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intens_color(cloud, "intensity");
	viewer.addPointCloud<pcl::PointXYZRGB> (rgb_cloud, rgb, "input_cloud");
	//std::cout << "afteer addPointCloud" << std::endl;
	viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);
	//viewer.setCameraPosition(-4.5, 0, 0.86, 0, 0, 1, 0);

	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
	//viewer.addCoordinateSystem (1.0);
	//viewer.initCameraParameters ();

	/*
	if (nullptr != fullObjectList)
	{
		ROS_INFO("ObjectList not NULL!");
		int num = 1;
		for (auto& obj:fullObjectList->ObjectList)
		{
			viewer.removeShape("cube_"+std::to_string(num));
			viewer.removeText3D("text_"+std::to_string(num));
			float r=(rand()%100)/100.0;
			float g=(rand()%100)/100.0;
			float b=(rand()%100)/100.0;
			viewer.addCube(obj.x.min, obj.x.max, obj.y.min, obj.y.max, obj.z.min, obj.z.max, r, g, b, "cube_"+std::to_string(num));
			viewer.addText3D<pcl::PointXYZ>("Object_"+std::to_string(num)+", class="+std::to_string(obj.classNr),
			pcl::PointXYZ(obj.x.mean, obj.y.mean, obj.z.mean),
			0.2,
			r, g, b,
			"text_"+std::to_string(num));
			num++;
		}
		viewer.spinOnce();
		//std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	*/
	viewer.spinOnce();
	// viewer.spin();
	//std::this_thread::sleep_for(std::chrono::seconds(1));
}

void objCallback(const run_basics::ObjectList::ConstPtr& data)
{
	ROS_INFO("Recived new ObjectList!");

	fullObjectList = data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FinalVisualizer");

	ros::NodeHandle nh;

	subCloud = nh.subscribe("/BugaSegm/pc_segmented", 1, cloudCallback);
	subObjects = nh.subscribe("/BugaSegm/objectlist", 1, objCallback);

	ros::spin();

	return 0;
}