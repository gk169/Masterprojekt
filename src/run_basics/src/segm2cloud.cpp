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
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "../include/run_basics/organize_velodyne_cloud.h"

//typedef pcl::PointXYZ PointType;
#define PI 3.14159265

ros::Subscriber subCroppedCloud;
ros::Subscriber subImg;
ros::Publisher segmCloud_pub;

cv::Mat latest_img;// = cv::Mat::zeros(540, 960, CV_8UC3);
std::mutex mtxImg;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

int getYID(velodyne_pointcloud::PointXYZIR point)
{
    float b = 2.0*point.x;
    float width = 480;//960;

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
    float d = 2.0*tan(25.0 * PI / 180.0)*point.x;
    float height = 360;//540;

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

    velodyne_pointcloud::PointXYZIR ret_point;

    ret_point.x = sX + point.x;
    ret_point.y = sY + point.y;
    ret_point.z = sZ + point.z;
    ret_point.ring = point.ring;
    ret_point.intensity = point.intensity;

    return ret_point;
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

int getSegmFromImg(float x, float y)
{
    //std::cout << (int)round(x) << "+++" << (int)round(y) << std::endl;
    cv::Vec3b colour = latest_img.at<cv::Vec3b>((int)round(y), (int)round(x));
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
//        std::cout << "SEGMENT IS -1! -> bgr not found: " << std::to_string(bgr) << std::endl;
//        std::cout << std::to_string(colour[2]) << " - " << std::to_string(colour[1]) << " - " << std::to_string(colour[0]) << std::endl;
//        std::cout << std::to_string(x) << " - " << std::to_string(y) << std::endl << std::endl;
    }

    return seg_num;
}

int getR(float x, float y)
{
    //std::cout << (int)round(x) << "+++" << (int)round(y) << std::endl;
    cv::Vec3b colour = latest_img.at<cv::Vec3b>((int)round(y), (int)round(x));

    return colour[0];
}
int getG(float x, float y)
{
    //std::cout << (int)round(x) << "+++" << (int)round(y) << std::endl;
    cv::Vec3b colour = latest_img.at<cv::Vec3b>((int)round(y), (int)round(x));

    return colour[1];
}
int getB(float x, float y)
{
    //std::cout << (int)round(x) << "+++" << (int)round(y) << std::endl;
    cv::Vec3b colour = latest_img.at<cv::Vec3b>((int)round(y), (int)round(x));

    return colour[2];
}

void addSegmToCloud(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud)
{
//    CamA.at<float>(2,2) = 1;
    double fWidth_px = 1157.519519/4; //1139.081372/4;
    double fHeight_px = 1158.977934/4; //1140.301608/4;
    double principal_x = 931.165229/4; //932.960706/4;
    double principal_y = 529.569982/4; //513.207084/4;

    cv::Mat CamA = (cv::Mat_<double>(3,3) << fWidth_px, 0, principal_x, 0, fHeight_px, principal_y, 0, 0, 1);
    //cv::Mat CamA = (cv::Mat_<double>(3,3) << fWidth_px, 0, 0, 0, fHeight_px, 0, principal_x, principal_y, 1);

//    CamA.at<float>(0,0) = fWidth_px;
//    CamA.at<float>(1,1) = fHeight_px;
//    CamA.at<float>(0,2) = principal_x;
//    CamA.at<float>(1,2) = principal_y;

    float PCL_X, PCL_Y, PCL_Z, PCL_Roll, PCL_Yaw, PCL_Pitch;
    ros::param::get("/PCL_X", PCL_X);
    ros::param::get("/PCL_Y", PCL_Y);
    ros::param::get("/PCL_Z", PCL_Z);
    ros::param::get("/PCL_Roll", PCL_Roll);
    ros::param::get("/PCL_Pitch", PCL_Pitch);
    ros::param::get("/PCL_Yaw", PCL_Yaw);
    float sX = -0.95; //Distance Camera System to Top Lidar System! -0.95, 0, 0.53
    float sY = 0;
    float sZ = 0.53;
    //cv::Mat rvec = (cv::Mat_<double>(3,3) << cos(PCL_Yaw * PI / 180.0), (sin(PCL_Yaw * PI / 180.0)), 0, -(sin(PCL_Yaw * PI / 180.0)), cos(PCL_Yaw * PI / 180.0), 0, 0, 0, 1);
    //cv::Mat rvec = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, cos(PCL_Yaw * PI / 180.0), (sin(PCL_Yaw * PI / 180.0)), 0, -(sin(PCL_Yaw * PI / 180.0)), cos(PCL_Yaw * PI / 180.0));
    cv::Mat rvec = (cv::Mat_<double>(3,3) << cos(PCL_Yaw * PI / 180.0), 0, -(sin(PCL_Yaw * PI / 180.0)), 0, 1, 0, (sin(PCL_Yaw * PI / 180.0)), 0, cos(PCL_Yaw * PI / 180.0));
    cv::Mat tvec = (cv::Mat_<double>(3,1) << -sY+PCL_Y, -sZ+PCL_Z, sX+PCL_X);

    std::cout << "RVEC: " << std::endl;
    std::cout << rvec << std::endl;

    vector<cv::Point3f> cloud_Vec;
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud_Vec.push_back(cv::Point3f(-(cloud->points[i].y),-(cloud->points[i].z),cloud->points[i].x));
    }

    //Add rotation to adjust camera to Lidars --> ~2° like matrix in cloudCrop
    float shiftAngle = 1;
    //float pX = (cos(shiftAngle * PI / 180.0))*point.x-(sin(shiftAngle * PI / 180.0))*point.y; //Position of Point, rotated 45°!
    //float pY = (sin(shiftAngle * PI / 180.0))*point.x+(cos(shiftAngle * PI / 180.0))*point.y;
    //cv::Mat rvec = (cv::Mat_<double>(3,3) << cos(shiftAngle * PI / 180.0), (sin(shiftAngle * PI / 180.0)), 0, -(sin(shiftAngle * PI / 180.0)), cos(shiftAngle * PI / 180.0), 0, 0, 0, 1);
//    cv::Mat rvec = (cv::Mat_<double>(3,1) << (0, 0, 1) * (shiftAngle * PI / 180.0));
//    cv::Mat R;
//    cv::Rodrigues(rvec, R);
//    cv::Mat rvec = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    //cv::Mat tvec = (cv::Mat_<double>(3,1) << 0,0,0);
    //cv::Mat tvec = (cv::Mat_<double>(3,1) << -sY, -sZ, sX);

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0.050773; //0.056972;
    distCoeffs.at<double>(1) = -0.106031; //-0.114493;
    distCoeffs.at<double>(2) = -0.001663; //-0.001890;
    distCoeffs.at<double>(3) = 0.000080; //-0.002819;
    distCoeffs.at<double>(4) = 0;

    vector<cv::Point2f> image_points;
    cv::projectPoints(cloud_Vec, rvec /*cv::Mat::eye(3, 3, CV_64F)*/, tvec, CamA, /*cv::noArray()*/distCoeffs, image_points);

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        //std::cout << cloud_Vec[i].x << " --- " << cloud_Vec[i].y << " --- " << cloud_Vec[i].z << std::endl;
        //std::cout << rvec.at<double>(1,1) << " --- " << rvec.at<double>(0,2) << std::endl;
        //std::cout << image_points[i].x << " --- " << image_points[i].y << std::endl;
        //cloud->points[i].intensity = getSegmFromImg(image_points[i].x, image_points[i].y); //Old impl where bgr image was received

        int x = (int)round(image_points[i].x);
        int y = (int)round(image_points[i].y);

        if (0>x || 480<x || 0>y || 270<y)
        {
            cloud->points[i].ring = 0;
            cloud->points[i].intensity = 12;
        }
        else
        {
            cv::Vec3b bgr = latest_img.at<cv::Vec3b>(y, x);
            cloud->points[i].ring = (int)bgr[2];
            cloud->points[i].intensity = (int)bgr[1];
        }
//        std::cout << image_points[i].x << " --- " << image_points[i].y << std::endl;
//        std::cout << "intens: " << std::to_string(bgr[1]) << "---: " << std::to_string(bgr[0]) << "--: " << std::to_string(bgr[2]) << std::endl;
    }

//    static bool first = true;
//    if (first)
//    {
//        //viewer.setCameraPosition(0.95,0,-0.53,0,-1,0,0);
//        viewer.initCameraParameters ();
//        viewer.addCoordinateSystem (1.0);
////        ros::param::set("/CAM_PosX", -4.5);
////        ros::param::set("/CAM_PosY", 0);
////        ros::param::set("/CAM_PosZ", 0.86);
////        ros::param::set("/CAM_OrtX", 0);
////        ros::param::set("/CAM_OrtY", 0);
////        ros::param::set("/CAM_OrtZ", 1);

//        ros::param::set("/PCL_X", 0);
//        ros::param::set("/PCL_Y", 0);
//        ros::param::set("/PCL_Z", 0);
//        ros::param::set("/PCL_Roll", 0);
//        ros::param::set("/PCL_Pitch", 0);
//        ros::param::set("/PCL_Yaw", 0);
//        while(true)
//        {
//            float CAM_PosX, CAM_PosY, CAM_PosZ, CAM_OrtX, CAM_OrtY, CAM_OrtZ;
//            float PCL_X, PCL_Y, PCL_Z, PCL_Roll, PCL_Yaw, PCL_Pitch;
//            ros::param::get("/CAM_PosX", CAM_PosX);
//            ros::param::get("/CAM_PosY", CAM_PosY);
//            ros::param::get("/CAM_PosZ", CAM_PosZ);
//            ros::param::get("/CAM_OrtX", CAM_OrtX);
//            ros::param::get("/CAM_OrtY", CAM_OrtY);
//            ros::param::get("/CAM_OrtZ", CAM_OrtZ);

//            ros::param::get("/PCL_X", PCL_X);
//            ros::param::get("/PCL_Y", PCL_Y);
//            ros::param::get("/PCL_Z", PCL_Z);
//            ros::param::get("/PCL_Roll", PCL_Roll);
//            ros::param::get("/PCL_Pitch", PCL_Pitch);
//            ros::param::get("/PCL_Yaw", PCL_Yaw);
//            first = false;

//            rvec = (cv::Mat_<double>(3,3) << cos(PCL_Yaw * PI / 180.0), (sin(PCL_Yaw * PI / 180.0)), 0, -(sin(PCL_Yaw * PI / 180.0)), cos(PCL_Yaw * PI / 180.0), 0, 0, 0, 1);
//            cv::Mat tvec = (cv::Mat_<double>(3,1) << -sY+PCL_Y, -sZ+PCL_Z, sX+PCL_X);

//            image_points.clear();
//            cv::projectPoints(cloud_Vec, rvec /*cv::Mat::eye(3, 3, CV_64F)*/, tvec, CamA, /*cv::noArray()*/distCoeffs, image_points);

//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
//            for (std::size_t i = 0; i < cloud->points.size (); ++i)
//            {
//                pcl::PointXYZRGB p;
//                p.x = cloud->points[i].x;
//                p.y = cloud->points[i].y;
//                p.z = cloud->points[i].z;
//                cv::Vec3b bgr = latest_img.at<cv::Vec3b>(image_points[i].y, image_points[i].x);
//                //(int)bgr[1];
//                p.r = ((int)bgr[1])/12*255;
//                p.g = ((int)bgr[1])/12*255;
//                p.b = ((int)bgr[1])/12*255;
////                p.r = getR(image_points[i].x, image_points[i].y);
////                p.g = getG(image_points[i].x, image_points[i].y);
////                p.b = getB(image_points[i].x, image_points[i].y);
//                cloud_rgb->points.push_back(p);
//            }
//            // Display pointcloud:
//            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
//            viewer.addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, "input_cloud");
//            viewer.setCameraPosition(CAM_PosX,CAM_PosY,CAM_PosZ,CAM_OrtX,CAM_OrtY,CAM_OrtZ,0);

//            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
//            //viewer.addCoordinateSystem (1.0);
//            //viewer.initCameraParameters ();

//            viewer.spinOnce();
//            std::this_thread::sleep_for(std::chrono::seconds(1));

//            viewer.removePointCloud("input_cloud");
////            float deg;
////            cout << "Degr around Z:";
////            cin >> deg;
////            cout << "Degr around Z: " << deg << std::endl;
////            rvec = (cv::Mat_<double>(3,3) << cos(deg * PI / 180.0), (sin(deg * PI / 180.0)), 0, -(sin(deg * PI / 180.0)), cos(deg * PI / 180.0), 0, 0, 0, 1);

////            image_points.clear();
////            cv::projectPoints(cloud_Vec, rvec /*cv::Mat::eye(3, 3, CV_64F)*/, tvec, CamA, /*cv::noArray()*/distCoeffs, image_points);

//        }
//    }
}

void veloCallback(const sensor_msgs::PointCloud2::ConstPtr& data)
{
    //ROS_INFO("Recived new pointCloud!");

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

//    mtxImg.lock();
//    for (std::size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        int colour = getColourFromImage(cloud->points[i]);

//        cloud->points[i].intensity = colour;
//    }
//    mtxImg.unlock();

//    for (std::size_t i = 0; i < cloud->points.size (); ++i)
//    {
//        cloud->points[i] = transfToCam(cloud->points[i]);
//    }

    if (!latest_img.empty())
    {
        addSegmToCloud(cloud);

        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud.get(),out_msg );

        segmCloud_pub.publish(out_msg);
    }
}

void imgCallback(const sensor_msgs::ImageConstPtr& data)
{
    //ROS_INFO("Recived new Image!");

    //mtxImg.lock();
    try
    {
        //cv::resize(cv_bridge::toCvShare(data, "rgb8")->image, latest_img, latest_img.size());
        latest_img = cv_bridge::toCvShare(data, "rgb8")->image;
        //cv::imshow("latest", latest_img);
        //cv::waitKey(0);
        //std::cout << latest_img.at<cv::Vec3b>(180, 240) << std::endl;
        //std::cout << std::to_string(latest_img.size().width) << " --- " << std::to_string(latest_img.size().height) << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", data->encoding.c_str());
    }
    //mtxImg.unlock();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segm2cloud");

    ros::NodeHandle nh;

    subCroppedCloud = nh.subscribe("/cropped_velodyne_points", 1000, veloCallback);
    subImg = nh.subscribe("/img_obj", 1000, imgCallback);
    //subImg = nh.subscribe("/static_image/image_raw", 1000, imgCallback);
    segmCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/segm_velodyne_points", 1000);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
