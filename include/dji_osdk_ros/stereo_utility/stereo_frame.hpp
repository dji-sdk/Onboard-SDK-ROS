#ifndef ONBOARDSDK_STEREO_FRAME_H
#define ONBOARDSDK_STEREO_FRAME_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "frame.hpp"
#include "camera_param.hpp"
#include "dji_ack.hpp"
#include "dji_log.hpp"
#include "point_cloud_viewer.hpp"

#ifdef USE_GPU
  #include <opencv2/cudastereo.hpp>
  #include <opencv2/cudawarping.hpp>
#endif

#ifdef USE_OPEN_CV_CONTRIB
  #include <opencv2/ximgproc/disparity_filter.hpp>
#endif

#include "sensor_msgs/Image.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"

#ifdef USE_DARKNET_ROS
#include "darknet_ros_msgs/BoundingBoxes.h"
#endif

namespace M210_STEREO
{

class StereoFrame
{
public:
  typedef std::shared_ptr<StereoFrame> Ptr;

  StereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam,
              int num_disp = 64, int block_size = 13);
  ~StereoFrame();

public:
  static StereoFrame::Ptr createStereoFrame(CameraParam::Ptr left_cam,
                                            CameraParam::Ptr right_cam);

  void readStereoImgs(const DJI::OSDK::ACK::StereoVGAImgData &imgs);

  void rectifyImgs();

  void computeDisparityMap();

  void filterDisparityMap();

  void unprojectPtCloud();

  void unprojectROSPtCloud();

#ifdef USE_DARKNET_ROS
  void calcObjectInfo(const darknet_ros_msgs::BoundingBoxesConstPtr &b_box, visualization_msgs::MarkerArray &marker_array);
#endif

  inline cv::Mat getRectLeftImg() { return this->rectified_img_left_; }

  inline cv::Mat getRectRightImg() { return this->rectified_img_right_; }

  inline cv::Mat getDisparityMap() { return this->disparity_map_8u_; }

  inline cv::viz::WCloud getPtCloud() { return this->pt_cloud_; }

  inline sensor_msgs::PointCloud2 getROSPtCloud() { return this->ros_pt_cloud_; }

#ifdef USE_OPEN_CV_CONTRIB
  inline cv::Mat getFilteredDispMap() { return this->filtered_disparity_map_8u_; }
#endif

protected:
  bool initStereoParam();

protected:
  //! Image frames
  Frame::Ptr frame_left_ptr_;
  Frame::Ptr frame_right_ptr_;

  //! Camera related
  CameraParam::Ptr camera_left_ptr_;
  CameraParam::Ptr camera_right_ptr_;

  //! Stereo camera parameters
  cv::Mat param_rect_left_;
  cv::Mat param_rect_right_;
  cv::Mat param_proj_left_;
  cv::Mat param_proj_right_;

  cv::Mat rectified_mapping_[2][2];

  //! Rectified images
  cv::Mat rectified_img_left_;
  cv::Mat rectified_img_right_;

  //! Block matching related
  int num_disp_;
  int block_size_;
  cv::Ptr<cv::StereoBM> block_matcher_;
  cv::Mat disparity_map_8u_;
  cv::Mat raw_disparity_map_;

  //! Point Cloud related
  double principal_x_;
  double principal_y_;
  double fx_;
  double fy_;
  double baseline_x_fx_;

  // due to rectification, the image boarder are blank
  // we cut them out
  const int     border_size_;
  const int     trunc_img_width_end_;
  const int     trunc_img_height_end_;
  std::vector<uint8_t>  color_buffer_;
  cv::Mat               color_mat_;
  cv::Mat_<cv::Vec3f>   mat_vec3_pt_;
  cv::viz::WCloud       pt_cloud_;

#ifdef USE_GPU
  cv::cuda::GpuMat  cuda_rectified_mapping_[2][2];

  cv::cuda::GpuMat  cuda_rect_src;
  cv::cuda::GpuMat  cuda_rectified_img_left_;
  cv::cuda::GpuMat  cuda_rectified_img_right_;
  cv::cuda::GpuMat  cuda_disparity_map_8u;
#endif

#ifdef USE_OPEN_CV_CONTRIB
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
  cv::Ptr<cv::StereoMatcher> right_matcher_;

  cv::Mat raw_right_disparity_map_;
  cv::Mat filtered_disparity_map_;
  cv::Mat filtered_disparity_map_8u_;
#endif

  //! ROS related
  sensor_msgs::PointCloud2 ros_pt_cloud_;
  sensor_msgs::PointCloud2Modifier *cloud_modifier_;
  sensor_msgs::PointCloud2Iterator<float>   *x_it_;
  sensor_msgs::PointCloud2Iterator<float>   *y_it_;
  sensor_msgs::PointCloud2Iterator<float>   *z_it_;
  sensor_msgs::PointCloud2Iterator<uint8_t> *r_it_;
  sensor_msgs::PointCloud2Iterator<uint8_t> *g_it_;
  sensor_msgs::PointCloud2Iterator<uint8_t> *b_it_;

  visualization_msgs::Marker marker_template_;
};

} // namespace M210_STEREO
#endif //ONBOARDSDK_STEREO_FRAME_H
