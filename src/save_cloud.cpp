#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static bool save_img = false;
static bool terminate = false;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  int save_num =0;


public:
  ImageConverter()
    : it_(nh_)
      
  {
    image_sub_ = it_.subscribe("/nerian_stereo/depth_map", 1,
      &ImageConverter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Update GUI Window
    if(save_img){
        ROS_INFO("saved image");
        cv::FileStorage file("depth" + std::to_string(save_num) + ".xml", cv::FileStorage::WRITE);
        save_num++;
        cv::Mat image = cv_ptr->image;
        file << "matName" << image;
        save_img = false;
    }

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }
};
void keyboard_loop(){
    char c;
    while(ros::ok()){
    c = std::cin.get();
    if(c == 's'){save_img =true;}
    }
}


void video_loop(){
    ImageConverter ic;
    while(ros::ok()){
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  std::thread image_loop(video_loop);
  std::thread keyboard(keyboard_loop);
  keyboard.join();
  image_loop.join();
  std::cout << "exited convereter" << std::endl;
  //ros::spin();
  return 0;
}
