#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/PointCloud2.h"

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher velodyne_front_pub;
ros::Publisher velodyne_top_pub;
ros::Publisher velodyne_back_pub;
ros::Publisher img_compressed_pub;


void callback(const PointCloud2::ConstPtr& velodyne_front, const PointCloud2::ConstPtr& velodyne_top, const PointCloud2::ConstPtr& velodyne_back, const CompressedImageConstPtr& img_compressed)
{
  // Solve all of perception here...
  ROS_INFO("Synchronizer - Sync_Callback");

  bool SEG_RUNNING = false;
  ros::param::get("/SEG_RUNNING", SEG_RUNNING);
  if(!SEG_RUNNING)
  {
    ROS_INFO("Synchronizer - Sync_Callback not blocked anymore, publishing synchronized data");
    ros::param::set("/SEG_RUNNING", true);
    //ROS_INFO("front TimeStamp: %d --- %d", velodyne_front->header.stamp.sec, velodyne_front->header.stamp.nsec);
    //ROS_INFO("top TimeStamp: %d --- %d", velodyne_top->header.stamp.sec, velodyne_top->header.stamp.nsec);
    //ROS_INFO("img TimeStamp: %d --- %d", img_compressed->header.stamp.sec, img_compressed->header.stamp.nsec);
    img_compressed_pub.publish(img_compressed);
    velodyne_front_pub.publish(velodyne_front);
    velodyne_top_pub.publish(velodyne_top);
    velodyne_back_pub.publish(velodyne_back);
  }
  else
  {
    ROS_INFO("Synchronizer - Sync_Callback still blocked");
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Synchronizer");

  std::string velodyne_front_topic = "/velodyne/front/velodyne_points";
  std::string velodyne_top_topic = "/velodyne/top/velodyne_points";
  std::string velodyne_back_topic = "/velodyne/back/velodyne_points";
  std::string img_compressed_topic = "/usb_cam/image_raw/compressed";


  ros::NodeHandle nh;
  message_filters::Subscriber<PointCloud2> velodyne_front_sub(nh, velodyne_front_topic, 100);
  message_filters::Subscriber<PointCloud2> velodyne_top_sub(nh, velodyne_top_topic, 100);
  message_filters::Subscriber<PointCloud2> velodyne_back_sub(nh, velodyne_back_topic, 100);
  message_filters::Subscriber<CompressedImage> img_compressed_sub(nh, img_compressed_topic, 100);

  velodyne_front_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/synchronized" + velodyne_front_topic, 1000);
  velodyne_top_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/synchronized" + velodyne_top_topic, 1000);
  velodyne_back_pub = nh.advertise<sensor_msgs::PointCloud2>("/BugaSegm/synchronized" + velodyne_back_topic, 1000);
  img_compressed_pub = nh.advertise<sensor_msgs::CompressedImage>("/BugaSegm/synchronized" + img_compressed_topic, 1000);

  typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, CompressedImage> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_front_sub, velodyne_top_sub, velodyne_back_sub, img_compressed_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
