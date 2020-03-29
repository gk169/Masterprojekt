#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
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

#include "../include/run_basics/organize_velodyne_cloud.h"

//typedef pcl::PointXYZ PointType;
#define PI 3.14159265

ros::Subscriber subVeloFront;
ros::Publisher croppedCloud_pub;

bool getInAngle(velodyne_pointcloud::PointXYZIR point)
{

    float sX = 0; //Distance Camera System to Lidar System!
    float sY = 0;
    float sZ = 0;
    float pX = point.x; //Position of Point!
    float pY = point.y;
    float pZ = point.z;

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

void veloCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
    ROS_INFO("Recived new pointCloud!");

//    std::cout << (*data).fields[0] << std::endl;
//    std::cout << (*data).fields[1] << std::endl;
//    std::cout << (*data).fields[2] << std::endl;
//    std::cout << (*data).fields[3] << std::endl;
//    std::cout << (*data).fields[4] << std::endl;
//    std::cout << (*data).fields[5] << std::endl;
//    std::cout << (*data).fields[6] << std::endl;
//    std::cout << (*data).fields[7] << std::endl;
//    std::cout << (*data).fields[8] << std::endl;
//    std::cout << (*data).fields[9] << std::endl;
//    std::cout << (*data).height << std::endl;
//    std::cout << (*data).width << std::endl;

//    pcl::PCLPointCloud2 pcl_pc2;
//    pcl_conversions::toPCL(*data,pcl_pc2);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
//    pcl::fromROSMsg(*data, *cloud);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

//    pcl::CropBox<pcl::PointXYZI> BoxFilter;

//    BoxFilter.setMin(Eigen::Vector4f(0, -5, -4, 1.0));
//    BoxFilter.setMax(Eigen::Vector4f(5, 5, 20, 1.0));
//    BoxFilter.setRotation(Eigen::Vector3f(0, 0, 43.244));
//    BoxFilter.setInputCloud(cloud);

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr dataOut( new pcl::PointCloud<velodyne_pointcloud::PointXYZIR> );
    //sensor_msgs::PointCloud2 dataOut;
    //BoxFilter.filter(*dataOut);

    pcl::copyPointCloud<velodyne_pointcloud::PointXYZIR>(*cloud, *dataOut);

    dataOut->width = 0;
    dataOut->height = 1;
    dataOut->points.clear();

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        if (getInAngle(cloud->points[i]))
        {
            dataOut->points.push_back(cloud->points[i]);
            dataOut->width = dataOut->width+1;
        }
    }

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*dataOut.get(),out_msg );

//    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_organized( new pcl::PointCloud<pcl::PointXYZI> );
//    OrganizePointCloud<pcl::PointXYZI>(out_msg, *pcl_organized.get(), 16);

//    sensor_msgs::PointCloud2 organized_out_msg;
//    pcl::toROSMsg(*pcl_organized.get(),organized_out_msg );

    croppedCloud_pub.publish(out_msg);
    //croppedCloud_pub.publish(organized_out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloudCrop");

    ros::NodeHandle nh;

    subVeloFront = nh.subscribe("/velodyne/front/velodyne_points", 1000, veloCallback);
    croppedCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cropped_velodyne_points", 1000);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
