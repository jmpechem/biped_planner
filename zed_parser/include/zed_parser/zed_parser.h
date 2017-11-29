#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class zed_parser{
  public:
  zed_parser();

  void imgCB(const sensor_msgs::Image::ConstPtr& img);
  void infoCB(const sensor_msgs::CameraInfo::ConstPtr& info);
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  private:
  ros::NodeHandle m_nh;
  ros::Subscriber zed_rect_image_sub;
  ros::Subscriber zed_cam_info_sub;
  ros::Subscriber zed_cam_cloud_sub;

  ros::Publisher rect_image_pub;
  ros::Publisher cam_info_pub;
  ros::Publisher cam_cloud_pub;

};
