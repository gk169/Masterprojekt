#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
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

void addSegmToCloud(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud, cv::Mat fullObjects)
{
    double fWidth_px = 1157.519519/4; //1139.081372/4;
    double fHeight_px = 1158.977934/4; //1140.301608/4;
    double principal_x = 931.165229/4; //932.960706/4;
    double principal_y = 529.569982/4; //513.207084/4;

    cv::Mat CamA = (cv::Mat_<double>(3,3) << fWidth_px, 0, principal_x, 0, fHeight_px, principal_y, 0, 0, 1);

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

    cv::Mat rvec = (cv::Mat_<double>(3,3) << cos(PCL_Yaw * PI / 180.0), 0, -(sin(PCL_Yaw * PI / 180.0)), 0, 1, 0, (sin(PCL_Yaw * PI / 180.0)), 0, cos(PCL_Yaw * PI / 180.0));
    cv::Mat tvec = (cv::Mat_<double>(3,1) << -sY+PCL_Y, -sZ+PCL_Z, sX+PCL_X);

    vector<cv::Point3f> cloud_Vec;
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud_Vec.push_back(cv::Point3f(-(cloud->points[i].y),-(cloud->points[i].z),cloud->points[i].x));
    }

    cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0.050773; //0.056972;
    distCoeffs.at<double>(1) = -0.106031; //-0.114493;
    distCoeffs.at<double>(2) = -0.001663; //-0.001890;
    distCoeffs.at<double>(3) = 0.000080; //-0.002819;
    distCoeffs.at<double>(4) = 0;

    vector<cv::Point2f> image_points;
    cv::projectPoints(cloud_Vec, rvec, tvec, CamA, distCoeffs, image_points);

    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        int x = (int)round(image_points[i].x);
        int y = (int)round(image_points[i].y);

        if (0>x || 480<x || 0>y || 270<y)
        {
            cloud->points[i].ring = 0;
            cloud->points[i].intensity = 12;
        }
        else
        {
            cv::Vec3b bgr = fullObjects.at<cv::Vec3b>(y, x);
            cloud->points[i].ring = (int)bgr[0];
            cloud->points[i].intensity = (int)bgr[1];
        }
    }
}

void callback(const ImageConstPtr& img_segm, const PointCloud2::ConstPtr& pc_preprocessed)
{
  // Solve all of perception here...
  ROS_INFO("Sync_Callback Combinator");

  cv::Mat latest_img;
    try
    {
        latest_img = cv_bridge::toCvShare(img_segm, "bgr8")->image;
        //cv::imshow("input_img", latest_img);
        //cv::waitKey(0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_segm->encoding.c_str());
    }
    unsigned int numOfObjectClasses = 3;
    std::vector<cv::Vec3b> bgrVals;
    bgrVals.push_back(cv::Vec3b(0, 64, 64));        //Person
    bgrVals.push_back(cv::Vec3b(128, 192, 192));    //Pfosten
    bgrVals.push_back(cv::Vec3b(0, 128, 128));      //Grünzeug
    bgrVals.push_back(cv::Vec3b(128, 128, 128));
    bgrVals.push_back(cv::Vec3b(0, 0, 128));
    bgrVals.push_back(cv::Vec3b(0, 69, 255));
    bgrVals.push_back(cv::Vec3b(128, 64, 128));
    bgrVals.push_back(cv::Vec3b(222, 40, 60));
    bgrVals.push_back(cv::Vec3b(128, 128, 192));
    bgrVals.push_back(cv::Vec3b(128, 64, 64));
    bgrVals.push_back(cv::Vec3b(128, 0, 64));
    bgrVals.push_back(cv::Vec3b(192, 128, 0));

    int thresh = 1;
    std::vector<cv::Mat> maskBGR;
    //std::vector<cv::Mat> resultBGR;
    for (auto& val : bgrVals)
    {
        //std::cout << std::to_string(val.val[0]) << " " << std::to_string(val.val[1]) << " " << std::to_string(val.val[2]) <<std::endl;
        cv::Scalar minBGR = cv::Scalar(val.val[0] - thresh, val.val[1] - thresh, val.val[2] - thresh);

        cv::Scalar maxBGR = cv::Scalar(val.val[0] + thresh, val.val[1] + thresh, val.val[2] + thresh);

        cv::Mat mask;
        //cv::Mat result;

        cv::inRange(latest_img, minBGR, maxBGR, mask);
        maskBGR.push_back(mask);

        //cv::bitwise_and(latest_img, latest_img, result, mask);
        //cv::imshow("input_img", latest_img);
        //cv::imshow("mask", mask);
        //cv::imshow("result", result);
        //cv::waitKey(5000);
    }

    //Show masks for Debug
//    int i = 1;
//    for (auto& val : maskBGR)
//    {
//        cv::imshow(std::string("mask"+std::to_string(i)), val);
//        i++;
//    }

    double minArea = 90;
    int objectNum = 1;
    int classNum = 0;
    cv::Mat fullObjects = cv::Mat::zeros(latest_img.rows, latest_img.cols, CV_8UC3);
    for (auto& mask : maskBGR)
    {
        if (classNum<numOfObjectClasses)
        {
            std::vector<std::vector<cv::Point>> AllContoursInObject;
            std::vector<cv::Vec4i> HierarchyOfAllContoursInObject;
            cv::findContours(mask, AllContoursInObject, HierarchyOfAllContoursInObject, cv::RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);

            int index=0;
            for (auto& cont : AllContoursInObject)
            {
                double StapleArea = cv::contourArea(cont);
                // Check min size
                if (StapleArea > minArea)
                {
                    cv::Mat resultMask(mask.rows, mask.cols, CV_8UC3, cv::Scalar(objectNum,classNum,0));
                    cv::Mat ObjectMask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
                    cv::drawContours(ObjectMask, AllContoursInObject, index, cv::Scalar(255, 255, 255), -1, 8, HierarchyOfAllContoursInObject);

                    cv::bitwise_and(resultMask, resultMask, fullObjects, ObjectMask);

                    //Print found Objects to input_image (DEBUG)
                    cv::Scalar randCol(rand()&255, rand()&255, rand()&255);
                    cv::drawContours(latest_img, AllContoursInObject, index, randCol, 1, 8, HierarchyOfAllContoursInObject);
                    cv::RotatedRect rect;
                    rect = cv::minAreaRect(AllContoursInObject[index]);
                    std::string objText = std::to_string(objectNum) + " -- " + std::to_string(classNum);
                    cv::putText(latest_img, objText, cv::Point(rect.center.x, rect.center.y), cv::FONT_HERSHEY_SIMPLEX, 0.4, randCol, 1);
//                    cv::imshow("ObjectMask", ObjectMask);
//                    cv::waitKey(0);
                    objectNum++;
                }
                index++;
            }
        }
        else
        {
            cv::Mat resultMask(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0,classNum,0));
            cv::bitwise_and(resultMask, resultMask, fullObjects, mask);

        }
        classNum++;
    }

  /* Finished with Image Processing, starting transformation to PointCloud */

  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pc_preprocessed, pcl_pc2);
  fromPCLPointCloud2(pcl_pc2, *cloud);

  addSegmToCloud(cloud, fullObjects);

  sensor_msgs::PointCloud2 out_msg;
  pcl::toROSMsg(*cloud.get(),out_msg );

  segmCloud_pub.publish(out_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Synchronizer");

  std::string img_segm_topic = "/BugaSegm/img_segm";
  std::string pc_preprocessed_topic = "/BugaSegm/pc_preprocessed";


  ros::NodeHandle nh;
  message_filters::Subscriber<Image> img_segm_sub(nh, img_segm_topic, 100);
  message_filters::Subscriber<PointCloud2> pc_preprocessed_sub(nh, pc_preprocessed_topic, 100);

  segmCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/pc_segmented", 1000);

  typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_segm_sub, pc_preprocessed_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
