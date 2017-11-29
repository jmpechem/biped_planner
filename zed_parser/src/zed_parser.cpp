#include <zed_parser/zed_parser.h>

zed_parser::zed_parser()
{
  zed_rect_image_sub = m_nh.subscribe<sensor_msgs::Image>("/zed/rgb/image_raw_color",100,&zed_parser::imgCB,this);
  zed_cam_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>("/zed/rgb/camera_info",100,&zed_parser::infoCB,this);
  zed_cam_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("/camera/point_cloud/cloud_registered",100,&zed_parser::cloudCB,this);

  rect_image_pub = m_nh.advertise<sensor_msgs::Image>("/cam/image_raw",100);
  cam_info_pub = m_nh.advertise<sensor_msgs::CameraInfo>("/cam/camera_info",100); 
  cam_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/cam/cloud",100);
}
void zed_parser::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
   sensor_msgs::PointCloud2 output_cloud;
   output_cloud.header = cloud->header;
   output_cloud.height = cloud->height;
   output_cloud.width = cloud->width;
   output_cloud.fields = cloud->fields;
   output_cloud.is_bigendian = cloud->is_bigendian;
   output_cloud.point_step = cloud->point_step;
   output_cloud.row_step = cloud->row_step;
   output_cloud.data = cloud->data;

   output_cloud.header.frame_id = "/base_link";
   cam_cloud_pub.publish(output_cloud);
}
void zed_parser::imgCB(const sensor_msgs::Image::ConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat   image;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    image = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image_target;
  cv::resize(image,image_target,cv::Size(640,480),0,0,cv::INTER_CUBIC);


  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;
  std_msgs::Header header; // empty header
  header.frame_id = "/base_link";
  header.seq = img->header.seq;
  header.stamp = img->header.stamp;
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_target);
  img_bridge.toImageMsg(img_msg);


  rect_image_pub.publish(img_msg);
}

void zed_parser::infoCB(const sensor_msgs::CameraInfo::ConstPtr& info)
{
 sensor_msgs::CameraInfo conv_info;
 conv_info.header = info->header;
 conv_info.header.frame_id = "/base_link";
 conv_info.height = 480;
 conv_info.width = 640;
 conv_info.distortion_model = info->distortion_model;
 conv_info.D = info->D;
 conv_info.K = info->K;
 conv_info.R = info->R;
 conv_info.P = info->P;
 conv_info.binning_x = info->binning_x;
 conv_info.binning_y = info->binning_y;
 conv_info.roi = info->roi;
 cam_info_pub.publish(conv_info);
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"zed_parser");
    ros::NodeHandle nh;
    zed_parser zed_make_it;
    ros::Rate rate(10.0);
    while(nh.ok())
      {
        ros::spinOnce();
      }
}
