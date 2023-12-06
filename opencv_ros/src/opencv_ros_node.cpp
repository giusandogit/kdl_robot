#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>        // Library added for step 1c

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //------------------------------------------------------------------ Step 1c ---------------------------------------------------------------------------------------
   
    Mat im;   // Mat element to store the frame of pixels
   
    cvtColor(cv_ptr->image, im,cv::COLOR_BGR2GRAY);   // Setting of a scale of grey to detect a circular BLOB in the frame 

    SimpleBlobDetector::Params params;
    params.minThreshold=0;
    params.maxThreshold=255;
    
    params.filterByCircularity=true;   // Set to true to look for circular objects
    params.minCircularity=0;           // I am looking for shapes from the least circular (e.g. triangles -> circularity = 0)...
    params.maxCircularity=1;           // ... to the most circular (e.g. circles -> circularity = 1)

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);  // Blob detector creation

    std::vector<cv::KeyPoint> keypoints;      // Blob detection and storage in the "detector->detect" field
    detector->detect(im, keypoints);

    drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);  // This line allows to draw a RED circle around the spotted circular object 
                                                                                                                           // (The "cv::Scalar()" function has the structure BGR, and not RGB, to define colors)

//-------------------------------------------------------------------- End of Step 1c -------------------------------------------------------------------------------

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}