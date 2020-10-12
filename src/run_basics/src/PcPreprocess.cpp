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

#include "../include/run_basics/organize_velodyne_cloud.h"

//typedef pcl::PointXYZ PointType;
#define PI 3.14159265

ros::Subscriber subVeloFront;
ros::Subscriber subVeloTop;
ros::Publisher croppedCloud_pub;

pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr TopCloud( new pcl::PointCloud<velodyne_pointcloud::PointXYZIR> );

bool getInAngle(velodyne_pointcloud::PointXYZIR point/*, bool front*/)
{

    float sX = -0.95; //Distance Camera System to Top Lidar System!
    float sY = 0;
    float sZ = 0.53;
//    if (front)
//    {
//        float pX = (sqrt(2)/2)*point.x-(sqrt(2)/2)*point.y; //Position of Point, rotated 45°!
//        float pY = (sqrt(2)/2)*point.x+(sqrt(2)/2)*point.y;
//        float pZ = point.z;
//    }
//    else
//    {
        float pX = point.x; //Position of Point!
        float pY = point.y;
        float pZ = point.z;
//    }
    float angleWidth = 45;
    float angleHeight = 25;
    static float tanWidth = tan(angleWidth  * PI / 180.0);
    static float tanHeight = tan(angleHeight  * PI / 180.0);

    float dX = sX + pX;
    float dY = sY + pY;
    float dZ = sZ + pZ;

    float rangeY = tanWidth*dX;
    float rangeZ = tanHeight*dX;

//    std::cout << "rangeY=" << rangeY << std::endl;
//    std::cout << "rangeZ=" << rangeZ << std::endl;

//    std::cout << "dX=" << dX << std::endl;
//    std::cout << "dY=" << dY << std::endl;
//    std::cout << "dZ=" << dZ << std::endl;

    //check width
    if (!(abs(dY) <= rangeY))
    {
        return false;
    }

    //check height
    if (!(abs(dZ) <= rangeZ))
    {
        return false;
    }

    return true;
}

velodyne_pointcloud::PointXYZIR transformFrontToTopVelo(velodyne_pointcloud::PointXYZIR point)
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

    velodyne_pointcloud::PointXYZIR ret_point;

    ret_point.x = sX + pX;
    ret_point.y = sY + pY;
    ret_point.z = sZ + pZ;
    ret_point.ring = point.ring;
    ret_point.intensity = point.intensity;

    return ret_point;
}

void veloFrontCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
    ROS_INFO("Recived new Front-pointCloud!");

    std_msgs::Header header = data->header;

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

    //pcl::io::savePCDFileASCII ("/home/segmentation/Git/Masterprojekt/front_cloud.pcd", *cloud.get());
    //std::cout << "---------------" << std::endl;
    //std::this_thread::sleep_for(std::chrono::seconds(3));

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr dataOut( new pcl::PointCloud<velodyne_pointcloud::PointXYZIR> );

    pcl::copyPointCloud<velodyne_pointcloud::PointXYZIR>(*TopCloud, *dataOut);

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr fullCloud( new pcl::PointCloud<velodyne_pointcloud::PointXYZIR> );

    //add top points to pointcloud
    pcl::copyPointCloud<velodyne_pointcloud::PointXYZIR>(*TopCloud, *fullCloud);

    //add front points to pointcloud
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        fullCloud->points.push_back(transformFrontToTopVelo(cloud->points[i]));
    }

    dataOut->width = 0;
    dataOut->height = 1;
    dataOut->points.clear();

    for (std::size_t i = 0; i < fullCloud->points.size (); ++i)
    {
        if (getInAngle(fullCloud->points[i]))
        {
            dataOut->points.push_back(fullCloud->points[i]);
            dataOut->width = dataOut->width+1;
        }
    }
//    for (std::size_t i = 0; i < TopCloud->points.size (); ++i)
//    {
//        if (getInAngle(TopCloud->points[i]))
//        {
//            dataOut->points.push_back(TopCloud->points[i]);
//            dataOut->width = dataOut->width+1;
//        }
//    }

//    pcl::io::savePCDFileASCII ("/home/micha/Masterprojekt/cropped_cloud.pcd", *dataOut.get());
//    std::cout << "---------------" << std::endl;
//    std::this_thread::sleep_for(std::chrono::seconds(3));

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*dataOut.get(),out_msg );

//    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_organized( new pcl::PointCloud<pcl::PointXYZI> );
//    OrganizePointCloud<pcl::PointXYZI>(out_msg, *pcl_organized.get(), 16);

//    sensor_msgs::PointCloud2 organized_out_msg;
//    pcl::toROSMsg(*pcl_organized.get(),organized_out_msg );

    out_msg.header=header;
    croppedCloud_pub.publish(out_msg);
    //croppedCloud_pub.publish(organized_out_msg);
}

void veloTopCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
    ROS_INFO("Recived new Top-pointCloud!");

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

    //pcl::io::savePCDFileASCII ("/home/segmentation/Git/Masterprojekt/top_cloud.pcd", *cloud.get());
    //std::cout << "------saved top---------" << std::endl;
    //std::this_thread::sleep_for(std::chrono::seconds(3));

    pcl::copyPointCloud<velodyne_pointcloud::PointXYZIR>(*cloud, *TopCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PcPreprocess");

    ros::NodeHandle nh;

    subVeloFront = nh.subscribe("/BugaSegm/synchronized/velodyne/front/velodyne_points", 1000, veloFrontCallback);
    subVeloTop = nh.subscribe("/BugaSegm/synchronized/velodyne/top/velodyne_points", 1000, veloTopCallback);
    croppedCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/pc_preprocessed", 1000);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
