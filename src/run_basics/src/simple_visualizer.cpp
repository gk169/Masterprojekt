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

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud,"intensity");
    viewer.removePointCloud("input_cloud");

    //viewer.addPointCloud<pcl::PointXYZI> (cloud, handler, "input_cloud");
    viewer.addPointCloud<pcl::PointXYZI> (cloud, "input_cloud");
    viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");

    while(true)
    {

        ros::param::get("/CAM_PosX", CAM_PosX);
        ros::param::get("/CAM_PosY", CAM_PosY);
        ros::param::get("/CAM_PosZ", CAM_PosZ);
        ros::param::get("/CAM_OrtX", CAM_OrtX);
        ros::param::get("/CAM_OrtY", CAM_OrtY);
        ros::param::get("/CAM_OrtZ", CAM_OrtZ);

        viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);

        viewer.spinOnce();

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}

int main(int argc, char **argv)
{

    std::string topic;
    if (argc<2)
    {
        std::cout << "No topic specified to display!" << std::endl << "Terminating!" << std::endl;
        return 0;
    }
    else
    {
        topic = argv[1];
        std::cout << "Displaying cloud from topic " << topic << std::endl;
    }

    ros::init(argc, argv, "simple_visualizer");

    ros::NodeHandle nh;

    subCloud = nh.subscribe(topic, 1000, cloudCallback);

    ros::spin();

    return 0;
}
