#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector> 
#include <string>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/features2d.hpp>
#include <camera_info_manager/camera_info_manager.h>

using namespace std;
using namespace sensor_msgs;

class IRNormalization
{
    public:
	    IRNormalization(ros::NodeHandle nh_);
	    ~IRNormalization();
       
    protected:
        void imageCB(const sensor_msgs::ImageConstPtr& msg);
    
        ros::NodeHandle nh_;    
        
        image_transport::ImageTransport    _imageTransport;
        image_transport::Subscriber  image_sub;
        image_transport::Publisher         image_pub;

        geometry_msgs::TransformStamped    transformStamped;
        image_geometry::PinholeCameraModel cam_model;

        //Ros topics names
        string strImage_sub_topic;
        string strImage_pub_topic;
        string strCameraInfo_sub_topic;

        //intrinsic matrix
        cv::Matx33d cameraMatrix;
        
        //Parameters for camera info
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        string strCameraModel;
};

IRNormalization::IRNormalization(ros::NodeHandle nh_): _imageTransport(nh_), 
                                                  cinfo_(new camera_info_manager::CameraInfoManager(nh_))
{       
    //Parameters for topics 
    nh_.param<std::string>("strImage_sub_topic", strImage_sub_topic, "/camera/ir/image");
    nh_.param<std::string>("strImage_pub_topic", strImage_pub_topic, "/orbbec/ir/image");
    nh_.param<std::string>("strCameraInfo_sub_topic", strCameraInfo_sub_topic, "/camera/ir/camera_info");
  
    nh_.param<std::string>("strCameraModel", strCameraModel, "");
    ROS_INFO("CamInfo Link: %s",strCameraModel.c_str());

    if (cinfo_->validateURL(strCameraModel))
    {
        cinfo_->loadCameraInfo(strCameraModel);
        ROS_INFO("Got camera info & loaded!");
        // cout <<  cinfo_->getCameraInfo();
    }
    else
        ROS_INFO("Recheck URL, stupid!!!!!!!");


    //publisher & subcriber 
    image_transport::TransportHints th;
    // image_sub = _imageTransport.subscribe(strImage_sub_topic, 1, &IRNormalization::imageCB, this, image_transport::TransportHints("")); 
    image_sub = _imageTransport.subscribe(strImage_sub_topic, 10, &IRNormalization::imageCB, this, th);   
    ROS_INFO("Subcribed to the topic: %s", strImage_sub_topic.c_str());

    image_pub = _imageTransport.advertise(strImage_pub_topic, 10);
    ROS_INFO("Published to the topic: %s", strImage_pub_topic.c_str());


}

IRNormalization::~IRNormalization()
{
	cv::destroyAllWindows();
}

void IRNormalization::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img, imgNormalized;
	cv_bridge::CvImagePtr cvPtr;
 	
    try
	{
        //convert msg to cvPtr
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
        
    cam_model.fromCameraInfo(cinfo_->getCameraInfo());
    cameraMatrix = cam_model.intrinsicMatrix();
   
    //copy value to cvMat
    cvPtr->image.copyTo(img);
    
  
    
    //publish the topic to ROS
    try
    {   
        //convert cv image to ROS message
        cv_bridge::CvImage image_msg;
        image_msg.header   = msg->header; // Same timestamp and tf frame as input image
        image_msg.encoding = sensor_msgs::image_encodings::MONO16; 
        image_msg.image  = img;
        cv::normalize(img,imgNormalized,0.,255.,cv::NORM_MINMAX,CV_8U);

        cv_bridge::CvImage normalize_msg;
        normalize_msg.header   = msg->header; // Same timestamp and tf frame as input image
        normalize_msg.encoding = sensor_msgs::image_encodings::MONO8; 
        normalize_msg.image    = imgNormalized;
        image_pub.publish(normalize_msg.toImageMsg());
    }
    catch  (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
    

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir_normalization");
  ros::NodeHandle nh;
  IRNormalization in(nh);
  ros::spin();
}