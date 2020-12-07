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

run_basics::ObjectList::ConstPtr ObjectList = nullptr;
sensor_msgs::PointCloud2::ConstPtr PC_Data = nullptr;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

float CAM_PosX, CAM_PosY, CAM_PosZ, CAM_OrtX, CAM_OrtY, CAM_OrtZ;
float PCL_X, PCL_Y, PCL_Z, PCL_Roll, PCL_Yaw, PCL_Pitch;

// Pointcloud colors
std::vector<std::vector<int>> bgrVals;
//std::vector<int> bgrVals;
bool bigSize = false;
bool objects = false;

void Visualize()
{
	if ((objects && nullptr == ObjectList) || nullptr == PC_Data) return;

	ROS_INFO("Visualizer - Both received, starting visualization");

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*PC_Data, pcl_pc2);
	fromPCLPointCloud2(pcl_pc2, *cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	for (std::uint64_t i = 0; i < cloud->points.size (); ++i)
	{
		pcl::PointXYZRGB point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		point.z = cloud->points[i].z;
		point.b = bgrVals[cloud->points[i].intensity][0];
		point.g = bgrVals[cloud->points[i].intensity][1];
		point.r = bgrVals[cloud->points[i].intensity][2];
		rgb_cloud->points.push_back(point);
	}

	viewer.removePointCloud("input_cloud");
	viewer.removeAllShapes();
	

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rgb_cloud);
	
	viewer.addPointCloud<pcl::PointXYZRGB> (rgb_cloud, rgb, "input_cloud");
     if (bigSize)
     {	
        	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
     }
		
	if(objects)
     {
        	int num = 1;
        	for (auto& obj : ObjectList->ObjectList)
        	{
        		float r=(rand()%100)/100.0;
        		float g=(rand()%100)/100.0;
        		float b=(rand()%100)/100.0;
        		viewer.addCube(obj.x.min, obj.x.max, obj.y.min, obj.y.max, obj.z.min, obj.z.max, r, g, b, "cube_"+std::to_string(num));
        		viewer.addText3D<pcl::PointXYZ>("Object_"+std::to_string(num)+", class="+std::to_string(obj.classNr),
        			pcl::PointXYZ(obj.x.mean, obj.y.mean, obj.z.mean), 0.2, r, g, b, "text_"+std::to_string(num));
        		num++;
        	}
     }        
    
    	ObjectList = nullptr;
	PC_Data = nullptr;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
	ROS_INFO("Recived new pointCloud!");

	PC_Data = data;

	Visualize();
}

void objCallback(const run_basics::ObjectList::ConstPtr& data)
{
	ROS_INFO("Recived new ObjectList!");

	ObjectList = data;

	Visualize();
}

std::vector<int> splitString2int(std::string input)
{
	std::istringstream ss(input);
	std::string token;

	std::vector<int> ret;
	while(std::getline(ss, token, ' ')) {
		ret.push_back(stoi(token));
	}

	return ret;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FinalVisualizer");

	ros::NodeHandle nh;

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

     // Shut GetOpt error messages down (return '?'): 
     opterr = 0;
     std::string topic;
     int opt;
     // Retrieve the options:
     while ( (opt = getopt(argc, argv, "ot:s")) != -1 ) {  // for each option...
        switch ( opt ) {
            case 't':
                    topic = optarg;
                break;
            case 'o':
                    objects = true;
                break;
            case 's':
                    bigSize = true;
                break;
            case '?':  // unknown option...
                    cerr << "Unknown option: '" << char(optopt) << "'!" << endl;
                break;
        }
     }

     std::vector<std::string> bgrVals_string;
     ros::param::get("/BGR_VALS", bgrVals_string);
     for (auto& vals : bgrVals_string)
     {
        bgrVals.push_back(splitString2int(vals));
     }

	viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);
	//viewer.addCoordinateSystem (1.0);

     subCloud = nh.subscribe(topic, 1, cloudCallback);
	//subCloud = nh.subscribe("/BugaSegm/pc_segm", 1, cloudCallback);
     //subCloud = nh.subscribe("/cloud_pcd", 1, cloudCallback);
	if(objects)
     {
         subObjects = nh.subscribe("/BugaSegm/objectlist", 1, objCallback);
     }

	while(ros::ok())
	{
		ros::spinOnce();
		viewer.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return 0;
}
