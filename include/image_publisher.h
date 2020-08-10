#ifndef INCLUDED_IMAGEPUBLISHER
#define INCLUDED_IMAGEPUBLISHER

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pylon/ImageEventHandler.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/GrabResultPtr.h>
#include <pylon/Pixel.h>
#include <pylon/PylonIncludes.h>
#include <pylon/PylonImage.h>
#include <ros/console.h>

namespace Pylon
{
  using namespace GenApi;
  using namespace std;

  class ImagePublisher : public CImageEventHandler
  {
  public:

    ImagePublisher(ros::NodeHandle nh, sensor_msgs::CameraInfo::Ptr cinfo, string frame_id, string topic_prefix = "", bool use_camera_time = false)
      : nh_(nh), it_(nh_), use_camera_time_(use_camera_time) {
      cam_pub_ = it_.advertiseCamera(topic_prefix + "image_raw", 1);
      converter_.OutputPixelFormat = PixelType_RGB8packed;
      cinfo_ = cinfo;
      frame_id_ = frame_id;
    }

    virtual void OnImagesSkipped( CInstantCamera& camera, size_t countOfSkippedImages)
    {
      ROS_INFO_STREAM (countOfSkippedImages  << " images have been skipped.");
    }

    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
      ROS_DEBUG_STREAM("*OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName());
      if (ptrGrabResult->GrabSucceeded())
      {
        converter_.Convert(pylon_image_, ptrGrabResult);

        cv::Mat cv_img_rgb = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)pylon_image_.GetBuffer());
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cv_img_rgb).toImageMsg();
        sensor_msgs::CameraInfo::Ptr cinfo = cinfo_;

        ros::Time timestamp = use_camera_time_ ? ros::Time().fromNSec(ptrGrabResult->GetTimeStamp())
          : ros::Time::now();

        image->header.frame_id = frame_id_;
        image->header.stamp = timestamp;
        cinfo->header.frame_id = frame_id_;
        cinfo->header.stamp = timestamp;

        cam_pub_.publish(image, cinfo);
      }
      else
      {
        ROS_ERROR_STREAM("Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
      }
    }
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher cam_pub_;
    sensor_msgs::CameraInfo::Ptr cinfo_;
    string frame_id_;

    CImageFormatConverter converter_;
    CPylonImage pylon_image_;
    EPixelType pixel_type_;
    bool use_camera_time_;
  };
}

#endif /* INCLUDED_IMAGEPUBLISHER */
