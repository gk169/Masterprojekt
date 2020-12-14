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

#define PI 3.14159265

ros::Subscriber subVeloFront;
ros::Subscriber subVeloTop;
ros::Subscriber subVeloBack;
ros::Publisher croppedCloud_pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr FrontCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr TopCloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr BackCloud;
std_msgs::Header header;

std::vector<float> front, back, top;

bool getInAngle(pcl::PointXYZI point, int angleStart, int angleEnd)
{
     float anglePoint = atan2(-point.y, point.x) * 180 / PI;
    
    //check angle
    if (anglePoint > angleStart && anglePoint < angleEnd)
        return true;

    return false;
}

void process_topics()
{
    if(nullptr == FrontCloud || nullptr == TopCloud || nullptr == BackCloud) return;

    ROS_INFO("PcFusion - All topics received, start processing!");

    pcl::PointCloud<pcl::PointXYZI> fullCloud;

    pcl::PointCloud<pcl::PointXYZI> FrontCloud_Cropped;
    pcl::PointCloud<pcl::PointXYZI> BackCloud_Cropped;

    for (std::size_t i = 0; i < FrontCloud->points.size (); ++i)
    {
        // cut 25° of border (missmatched in LiDAR-Net)
        if (getInAngle(FrontCloud->points[i], -165, 75))
        {
            FrontCloud_Cropped.points.push_back(FrontCloud->points[i]);
            FrontCloud_Cropped.width++;
        }
    }

    for (std::size_t i = 0; i < BackCloud->points.size (); ++i)
    {
        // cut 25° of border (missmatched in LiDAR-Net)
        if (!getInAngle(BackCloud->points[i], -110, 20))
        {
            BackCloud_Cropped.points.push_back(BackCloud->points[i]);
            BackCloud_Cropped.width++;
        }
    }

    Eigen::Affine3f transform_front = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_top = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_back = Eigen::Affine3f::Identity();

    transform_front.translation() << front[0], front[1], front[2];
    transform_top.translation() << top[0], top[1], top[2];
    transform_back.translation() << back[0], back[1], back[2];
    
    // Translate points to base
    pcl::transformPointCloud(FrontCloud_Cropped, FrontCloud_Cropped, transform_front);
    pcl::transformPointCloud(*TopCloud, *TopCloud, transform_top);
    pcl::transformPointCloud(BackCloud_Cropped, BackCloud_Cropped, transform_back);
    
    // Combine point clouds
    pcl::copyPointCloud<pcl::PointXYZI>(*TopCloud, fullCloud);
    fullCloud += FrontCloud_Cropped;
    fullCloud += BackCloud_Cropped;

    // Convert point clouds to msg
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(fullCloud, out_msg);
    out_msg.header=header;

    ROS_INFO("PcFusion - Publishing preprocessed-pointCloud!");
    croppedCloud_pub.publish(out_msg);

    FrontCloud = nullptr;
    TopCloud = nullptr;
    BackCloud = nullptr;
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

    // load params    
    ros::param::get("/BASE_TO_FRONT", front);
    ros::param::get("/BASE_TO_TOP", top);
    ros::param::get("/BASE_TO_BACK", back);

    ros::spin();

    return 0;
}
