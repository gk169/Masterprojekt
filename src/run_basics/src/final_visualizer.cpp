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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

    // Display pointcloud:

    viewer.removePointCloud("input_cloud");
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZ> (cloud, "input_cloud");
    viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);
    //viewer.setCameraPosition(-4.5, 0, 0.86, 0, 0, 1, 0);

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
    //viewer.addCoordinateSystem (1.0);
    //viewer.initCameraParameters ();

    if (nullptr != fullObjectList)
    {
        int num = 1;
        for (auto& obj:fullObjectList->ObjectList)
        {
            viewer.removeShape("cube_"+std::to_string(num));
            viewer.removeText3D("text_"+std::to_string(num));
            float r=(rand()%100)/100.0;
            float g=(rand()%100)/100.0;
            float b=(rand()%100)/100.0;
            viewer.addCube(obj.x.min, obj.x.max, obj.y.min, obj.y.max, obj.z.min, obj.z.max, r, g, b, "cube_"+std::to_string(num));
            viewer.addText3D<pcl::PointXYZ>("Object_"+std::to_string(num)+", class=",
                                        pcl::PointXYZ(obj.x.mean, obj.y.mean, obj.z.mean),
                                        0.2,
                                        r, g, b,
                                        "text_"+std::to_string(num));
            num++;
        }

        viewer.spin();
    }

   // viewer.spin();
    //std::this_thread::sleep_for(std::chrono::seconds(1));
}

void objCallback(const run_basics::ObjectList::ConstPtr& data)
{
    //ROS_INFO("Recived new ObjectList!");

    fullObjectList = data;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_visualizer");

    ros::NodeHandle nh;

    subCloud = nh.subscribe("/segm_velodyne_points", 1000, cloudCallback);
    subObjects = nh.subscribe("/pcl_segmentation/ObjectList", 1000, objCallback);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
