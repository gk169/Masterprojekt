#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "../include/run_basics/organize_velodyne_cloud.h"

#define PI 3.14159265

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher segmCloud_pub;

ImageConstPtr img_segm;
PointCloud2::ConstPtr pc_preprocessed;

std::vector<float> camera;

bool getInAngle(double coordinate1, double coordinate2, int angleStart, int angleEnd)
{
     float anglePoint = atan2(-coordinate2, coordinate1) * 180 / PI;

    //check angle
    if (anglePoint > angleStart && anglePoint < angleEnd)
        return true;

    return false;
}

void addSegmToCloud(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud, cv::Mat fullObjects)
{
    //#TODO: adapt camera parameters / add camera parameteres to sensor param file, make variables global?
    double fWidth_px = 1157.519519/4; //1139.081372/4;
    double fHeight_px = 1158.977934/4; //1140.301608/4;
    double principal_x = 931.165229/4; //932.960706/4;
    double principal_y = 529.569982/4; //513.207084/4;

    cv::Mat CamA = (cv::Mat_<double>(3,3) << fWidth_px, 0, principal_x, 0, fHeight_px, principal_y, 0, 0, 1);
    
    float sX = -camera[0];
    float sY = -camera[1];
    float sZ = -camera[2];
    
    float roll = camera[3];
    float pitch = camera[4];
    float yaw = camera[5];

    cv::Mat rvec_source = (cv::Mat_<double>(3,1) << pitch, yaw, roll);
    cv::Mat rvec;
    cv::Rodrigues(rvec_source, rvec);

    cv::Mat tvec = (cv::Mat_<double>(3,1) << -sY, -sZ, sX);

    cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << 0.050773, -0.106031, -0.001663, 0.000080, 0);
    //#TODO end

    vector<cv::Point3f> cloud_Vec;
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud_Vec.push_back(cv::Point3f(-(cloud->points[i].y),-(cloud->points[i].z),cloud->points[i].x));
    }
    
    vector<cv::Point2f> image_points;
    cv::projectPoints(cloud_Vec, rvec, tvec, CamA, distCoeffs, image_points);

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        int x = (int)round(image_points[i].x);
        int y = (int)round(image_points[i].y);
        cloud->points[i].ring = 0;

        //Check if: img-coord. are in image; point is ahead of camera; point is not near projection plane (distortion correction causes wrong projection)
        if (0>x || 480<=x || 0>y || 270<=y || cloud->points[i].x < -sX ||
                !getInAngle(cloud->points[i].x + sX, cloud->points[i].y + sY, -50, 50)||
                !getInAngle(cloud->points[i].x + sX, cloud->points[i].z + sZ, -30, 30))
        {
            //keep information from LiDAR_Net
            //cloud->points[i].ring = 0;
            cloud->points[i].intensity += 12;
        }
        else
        {
            int SegNet_class = fullObjects.at<uchar>(y, x);
            int LiDAR_class = cloud->points[i].intensity + 12;
            if ((SegNet_class == 3 || SegNet_class == 4 || SegNet_class == 5) && LiDAR_class == 14) //both class road
            {
                
                cloud->points[i].intensity = 15;
            }
            else if (SegNet_class == 3 || SegNet_class == 4 || SegNet_class == 5) //SegNet = road
            {
                //cloud->points[i].ring = 0;
                cloud->points[i].intensity = 17;
            }
            else if (LiDAR_class == 14) //LiDAR = road
            {
                //cloud->points[i].ring = 0;
                cloud->points[i].intensity = 16;
            }
            else
            {
                //cloud->points[i].ring = 0;
                cloud->points[i].intensity = SegNet_class;
            }
        }
    }
}

void process_topics()
{
    if (nullptr == img_segm || nullptr == pc_preprocessed) return;

    ROS_INFO("DataFusion - Both received, starting processing");

    cv::Mat latest_img;
    try
    {
        latest_img = cv_bridge::toCvShare(img_segm, "bgr8")->image;
        cv::cvtColor(latest_img, latest_img, cv::COLOR_BGR2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("DataFusion - Could not convert from '%s' to 'bgr8'.", img_segm->encoding.c_str());
    }

    cv::Mat fullObjects = cv::Mat::zeros(latest_img.rows, latest_img.cols, CV_8UC1);
    std::vector<cv::Mat> maskClass;
    for (uint8_t i=0; i<12; i++)
    {
        cv::Mat mask = cv::Mat::zeros(latest_img.rows, latest_img.cols, CV_8UC1);
        cv::Mat i_mask = cv::Mat(latest_img.rows, latest_img.cols, CV_8UC1, cv::Scalar(i));

        cv::compare(latest_img, i_mask, mask, cv::CMP_EQ);

        cv::Mat resultMask(mask.rows, mask.cols, CV_8UC1, i); //TODO one channel picture is enough
        cv::bitwise_and(resultMask, resultMask, fullObjects, mask);
    }

    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc_preprocessed, pcl_pc2);
    fromPCLPointCloud2(pcl_pc2, *cloud);

    addSegmToCloud(cloud, fullObjects);

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*cloud.get(),out_msg );

    img_segm = nullptr;
    pc_preprocessed = nullptr;

    segmCloud_pub.publish(out_msg);
}

void ImgCallback(const ImageConstPtr& img_segm_in)
{
    ROS_INFO("DataFusion - Received new Image in single CB");

    img_segm = img_segm_in;

    process_topics();
}

void PcCallback(const PointCloud2::ConstPtr& pc_in)
{
    ROS_INFO("DataFusion - Received new PC in single CB");

    pc_preprocessed = pc_in;

    process_topics();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DataFusion");

    std::string img_segm_topic = "/BugaSegm/img_segm";
    std::string pc_preprocessed_topic = "/BugaSegm/pc_segm";


    ros::NodeHandle nh;

    segmCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/pc_combined", 1000);

    //TimeSynchronizer not applicable because this needs more than one message for each topic!
    ros::Subscriber subImage = nh.subscribe(img_segm_topic, 1000, ImgCallback);
    ros::Subscriber subPc = nh.subscribe(pc_preprocessed_topic, 1000, PcCallback);

    // load params
    ros::param::get("/BASE_TO_CAMERA", camera);

    ros::spin();

    return 0;
}