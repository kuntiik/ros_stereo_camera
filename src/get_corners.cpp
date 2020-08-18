#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>

#include <pcl/point_types.h>
#include "pillar_corners.hpp"

double fx =672.8525634486431;
double fy = 672.758607019733;
double cx = 399.48117481367757;
double cy = 300.6428102963154;

int HEIGHT = 592;
int WIDTH = 800;

pcl::PointXYZ cv_to_pcl(cv::Point3i pt){
    pcl::PointXYZ p;
    p.x = ((double)pt.x - cx)/fx * (double)pt.z / 1000;
    p.y = ((double)pt.y - cy)/fy * (double)pt.z / 1000;
    p.z = (double)pt.z / 1000;
    return p;
}

void conver_to_cloud(cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int limit = 100000){
    cv::Point3i tmp;
    pcl::PointXYZ pt;
    cloud->clear();
    for(int i = 0; i < HEIGHT; i++){
        for(int j = 0; j < WIDTH; j++){
            tmp.x = j;
            tmp.y = i;
            tmp.z = image.at<uint16_t>(i,j);
            if(tmp.z != 65535 && tmp.z < limit){
                pt = cv_to_pcl(tmp);
                cloud->points.push_back(pt);
            }
        }
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher image_pub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners;
  sensor_msgs::PointCloud2 output_cloud;
  pcl::PCLPointCloud2 point_cloud2;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    cloud = (pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>);
    image_sub_ = it_.subscribe("/nerian_stereo/depth_map", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/image_converter/output_video", 1);
  }
  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    conver_to_cloud(cv_ptr->image, cloud, 15000);
    corners = pillar_corners_function(cloud);
    pcl::toPCLPointCloud2(*corners,point_cloud2);
    pcl_conversions::fromPCL(point_cloud2,output_cloud);
    output_cloud.header.frame_id="base_link";
    image_pub_.publish(output_cloud);
    //function to do all the stuf
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
