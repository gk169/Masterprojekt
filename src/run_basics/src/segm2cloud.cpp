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
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../include/run_basics/organize_velodyne_cloud.h"

//typedef pcl::PointXYZ PointType;
#define PI 3.14159265

ros::Subscriber subCroppedCloud;
ros::Subscriber subImg;
ros::Publisher segmCloud_pub;

cv::Mat latest_img;
std::mutex mtxImg;

int getYID(velodyne_pointcloud::PointXYZIR point)
{
    float b = 2*point.x;
    float width = 480;

    float pb = b/width;

    float numPixel = -(point.y/pb);

    int yID = (int)round(numPixel+width/2);

    if (480 == yID)
    {
        --yID;
    }
    return yID;
}

int getZID(velodyne_pointcloud::PointXYZIR point)
{
    float d = 2*tan(25.0 * PI / 180.0)*point.x;
    float height = 360;

    float ph = d/height;

    float numPixel = -(point.z/ph);

    int zID = (int)round(numPixel+height/2);


    if (360 == zID)
    {
        --zID;
    }
    return zID;
}

velodyne_pointcloud::PointXYZIR transfToCam(velodyne_pointcloud::PointXYZIR point)
{

    float sX = -0.95; //Distance Camera System to Top Lidar System!
    float sY = 0;
    float sZ = 0.53;

    point.x = sX + point.x;
    point.y = sY + point.y;
    point.z = sZ + point.z;

    return point;
}

int getColourFromImage(velodyne_pointcloud::PointXYZIR point)
{
    //std::cout << point.x << std::endl;
    velodyne_pointcloud::PointXYZIR pointTransf = transfToCam(point);
    //std::cout << pointTransf.x << std::endl;
    //std::cout << "---------" << std::endl;

    int yID = getYID(pointTransf);
    int zID = getZID(pointTransf);

    cv::Vec3b colour = latest_img.at<cv::Vec3b>(zID, yID);
    //std::cout << std::to_string(colour.val[0]) << " --- " << std::to_string(colour.val[1]) << " --- " << std::to_string(colour.val[2]) << std::endl;
    //cv::imshow( "Display window", latest_img );
    //cv::waitKey(0);

    unsigned int bgr = (colour[2] << 16) + (colour[1] << 8) + colour[0];
//    rgb = (rgb << 8) + colour[1];
//    rgb = (rgb << 8) + colour[0];
    int seg_num = -1;

    switch(bgr)
    {
    case(8421504):
    {
        seg_num = 0;
        break;
    }
    case(128):
    {
        seg_num = 1;
        break;
    }
    case(8437952):
    {
        seg_num = 2;
        break;
    }
    case(17919):
    {
        seg_num = 3;
        break;
    }
    case(8405120):
    {
        seg_num = 4;
        break;
    }
    case(14559292):
    {
        seg_num = 5;
        break;
    }
    case(32896):
    {
        seg_num = 6;
        break;
    }
    case(8421568):
    {
        seg_num = 7;
        break;
    }
    case(8405056):
    {
        seg_num = 8;
        break;
    }
    case(8388672):
    {
        seg_num = 9;
        break;
    }
    case(16448):
    {
        seg_num = 10;
        break;
    }
    case(12615680):
    {
        seg_num = 11;
        break;
    }
    }

    if (seg_num < 0)
    {
        std::cout << "SEGMENT IS -1! -> bgr not found: " << std::to_string(bgr) << std::endl;
        std::cout << std::to_string(colour[2]) << " - " << std::to_string(colour[1]) << " - " << std::to_string(colour[0]) << std::endl;
        std::cout << std::to_string(zID) << " - " << std::to_string(zID) << std::endl << std::endl;
    }

    return seg_num;
}

void veloCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
    //ROS_INFO("Recived new Front-pointCloud!");

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

//    int max_col = 0;
//    int zmax = 0, ymax=0, xmax=0;
//    int min_col = 999999999;
    mtxImg.lock();
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        int colour = getColourFromImage(cloud->points[i]);
        //std::cout << "col: " << std::to_string(colour) << "col/255: " << std::to_string(colour/255) << std::endl;

        cloud->points[i].intensity = colour;//255;
//        if (max_col < colour)
//        {
//            max_col = colour;
//        }
//        if (min_col > colour)
//        {
//            min_col = colour;
//            zmax = cloud->points[i].z;
//            ymax = cloud->points[i].y;
//            xmax = cloud->points[i].x;
//        }
    }
    mtxImg.unlock();
//    for (std::size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        cloud->points[i].intensity = cloud->points[i].intensity/max_col*255;
//    }
//    std::cout << "min: " << min_col << " --- max: " << max_col << std::endl;
//    std::cout << "MIN: zmax: " << zmax << " --- ymax: " << ymax << " --- xmax: " << xmax << std::endl;

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud.get(),out_msg );

    segmCloud_pub.publish(out_msg);
}

void imgCallback(const sensor_msgs::ImageConstPtr& data)
{
    //ROS_INFO("Recived new Image!");

    mtxImg.lock();
    try
    {
        latest_img = cv_bridge::toCvShare(data, "rgb8")->image;
        //std::cout << latest_img.at<cv::Vec3b>(180, 240) << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", data->encoding.c_str());
    }
    mtxImg.unlock();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segm2cloud");

    ros::NodeHandle nh;

    subCroppedCloud = nh.subscribe("/cropped_velodyne_points", 1000, veloCallback);
    //subImg = nh.subscribe("/img_segm", 1000, imgCallback);
    subImg = nh.subscribe("/image_publisher_1587475909812387642/image_raw", 1000, imgCallback);
    segmCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/segm_velodyne_points", 1000);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
