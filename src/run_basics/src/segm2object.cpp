#include "ros/ros.h"
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Subscriber subImg;
ros::Publisher ObjImg_pub;

void imgCallback(const sensor_msgs::ImageConstPtr& data)
{
    ROS_INFO("Recived new Image!");

    cv::Mat latest_img;
    try
    {
        latest_img = cv_bridge::toCvShare(data, "bgr8")->image;
        //cv::imshow("input_img", latest_img);
        //cv::waitKey(0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", data->encoding.c_str());
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

    cv_bridge::CvImage cv_imageBr;
    cv_imageBr.image = fullObjects;
    cv_imageBr.encoding = "bgr8";
    ObjImg_pub.publish(cv_imageBr.toImageMsg());

    //cv::imshow("latest_img", latest_img);
    //cv::imshow("fullObjects", fullObjects);
    //cv::waitKey(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segm2object");

    ros::NodeHandle nh;

    subImg = nh.subscribe("/img_segm", 1000, imgCallback);
    //subImg = nh.subscribe("/static_image/image_raw", 1000, imgCallback);
    ObjImg_pub = nh.advertise<sensor_msgs::Image>("/img_obj", 1000);

    ros::spin();

//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::seconds(10));
//        ros::spinOnce();
//    }

    return 0;
}
