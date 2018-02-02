#include <sensor_msgs/PointCloud2.h>
#include "stereo_utility/stereo_frame.hpp"

using namespace M210_STEREO;
using namespace cv;

StereoFrame::StereoFrame(CameraParam::Ptr left_cam,
                         CameraParam::Ptr right_cam,
                         int num_disp, int block_size)
  : camera_left_ptr_(left_cam)
  , camera_right_ptr_(right_cam)
  , num_disp_(num_disp)
  , block_size_(block_size)
  , color_buffer_(VGA_WIDTH*VGA_HEIGHT)
  , mat_vec3_pt_(VGA_HEIGHT, VGA_WIDTH, Vec3f(0, 0, 0))
  , color_mat_(VGA_HEIGHT, VGA_WIDTH, CV_8UC1, &color_buffer_[0])
  , pt_cloud_(mat_vec3_pt_, color_mat_)
  , raw_disparity_map_(Mat(VGA_HEIGHT, VGA_WIDTH, CV_16SC1))
  , border_size_(num_disp)
  , trunc_img_height_end_(VGA_HEIGHT - border_size_)
  , trunc_img_width_end_(VGA_WIDTH - border_size_)
{
  if(!this->initStereoParam())
  {
    ROS_ERROR("Failed to init stereo parameters\n");
  }

  Mat m210_vga_stereo_left  = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);
  Mat m210_vga_stereo_right = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);

  frame_left_ptr_   = Frame::createFrame(0, 0, m210_vga_stereo_left);
  frame_right_ptr_  = Frame::createFrame(0, 0, m210_vga_stereo_right);
}

StereoFrame::~StereoFrame()
{

}

bool
StereoFrame::initStereoParam()
{
  param_rect_left_ =  Config::get<Mat>("leftRectificationMatrix");
  param_rect_right_ = Config::get<Mat>("rightRectificationMatrix");
  param_proj_left_ =  Config::get<Mat>("leftProjectionMatrix");
  param_proj_right_ = Config::get<Mat>("rightProjectionMatrix");

  principal_x_ = param_proj_left_.at<double>(0, 2);
  principal_y_ = param_proj_left_.at<double>(1, 2);
  fx_ = param_proj_left_.at<double>(0, 0);
  fy_ = param_proj_left_.at<double>(1, 1);
  baseline_x_fx_ = -param_proj_right_.at<double>(0, 3);

  initUndistortRectifyMap(camera_left_ptr_->getIntrinsic(),
                          camera_left_ptr_->getDistortion(),
                          param_rect_left_,
                          param_proj_left_,
                          Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                          rectified_mapping_[0][0], rectified_mapping_[0][1]);
  initUndistortRectifyMap(camera_right_ptr_->getIntrinsic(),
                          camera_right_ptr_->getDistortion(),
                          param_rect_right_,
                          param_proj_right_,
                          Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                          rectified_mapping_[1][0], rectified_mapping_[1][1]);

#ifdef USE_GPU
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < 2; ++i) {
      cuda_rectified_mapping_[k][i].upload(rectified_mapping_[k][i]);
    }
  }

  block_matcher_ = cuda::createStereoBM(num_disp_, block_size_);
#else
  block_matcher_ = StereoBM::create(num_disp_, block_size_);
#endif

#ifdef USE_OPEN_CV_CONTRIB
  wls_filter_ = ximgproc::createDisparityWLSFilter(block_matcher_); // left_matcher
  wls_filter_->setLambda(8000.0);
  wls_filter_->setSigmaColor(1.5);

  right_matcher_ = ximgproc::createRightMatcher(block_matcher_);
#endif

  ros_pt_cloud_.header.frame_id = "map"; // Please change this accordingly
  ros_pt_cloud_.height = VGA_HEIGHT;
  ros_pt_cloud_.width = VGA_WIDTH;
  ros_pt_cloud_.point_step = sizeof(float)*3;
  ros_pt_cloud_.row_step = sizeof(float)*3*VGA_WIDTH;
  ros_pt_cloud_.is_bigendian = 0;
  ros_pt_cloud_.is_dense = 0;
  size_t num_pt_cloud = (trunc_img_height_end_-border_size_)*(trunc_img_width_end_-border_size_);
  // modifier resize this container
  ros_pt_cloud_.data.resize(num_pt_cloud);

  cloud_modifier_ = new sensor_msgs::PointCloud2Modifier(ros_pt_cloud_);
  cloud_modifier_->setPointCloud2FieldsByString(2, "xyz", "rgb");
  cloud_modifier_->resize(num_pt_cloud);

  x_it_ = new sensor_msgs::PointCloud2Iterator<float>(ros_pt_cloud_, "x");
  y_it_ = new sensor_msgs::PointCloud2Iterator<float>(ros_pt_cloud_, "y");
  z_it_ = new sensor_msgs::PointCloud2Iterator<float>(ros_pt_cloud_, "z");
  r_it_ = new sensor_msgs::PointCloud2Iterator<uint8_t>(ros_pt_cloud_, "r");
  g_it_ = new sensor_msgs::PointCloud2Iterator<uint8_t>(ros_pt_cloud_, "g");
  b_it_ = new sensor_msgs::PointCloud2Iterator<uint8_t>(ros_pt_cloud_, "b");


  //! Setup visualization info
  marker_template_.header.frame_id = "map"; // Please change this accordingly
  marker_template_.header.stamp = ros::Time();
  marker_template_.ns = "object";
  marker_template_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_template_.action = visualization_msgs::Marker::ADD;
  marker_template_.lifetime = ros::Duration(0.1);
  marker_template_.pose.orientation.x = 0.0;
  marker_template_.pose.orientation.y = 0.0;
  marker_template_.pose.orientation.z = 0.0;
  marker_template_.pose.orientation.w = 1.0;
  marker_template_.scale.x = 5;
  marker_template_.scale.y = 0.1;
  marker_template_.scale.z = 0.2;
  marker_template_.color.a = 1.0; // Don't forget to set the alpha!
  marker_template_.color.r = 0.0;
  marker_template_.color.g = 1.0;
  marker_template_.color.b = 0.0;


  return true;
}

StereoFrame::Ptr
StereoFrame::createStereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam)
{
  return std::make_shared<StereoFrame>(left_cam, right_cam);
}

void
StereoFrame::readStereoImgs(const sensor_msgs::ImageConstPtr &img_left, const sensor_msgs::ImageConstPtr &img_right)
{
  memcpy(frame_left_ptr_->raw_image.data,
         (char*)(&img_left->data[0]), sizeof(char)*VGA_HEIGHT*VGA_WIDTH);
  memcpy(frame_right_ptr_->raw_image.data,
         (char*)(&img_right->data[0]), sizeof(char)*VGA_HEIGHT*VGA_WIDTH);

  frame_left_ptr_ ->id  = img_left->header.seq;
  frame_right_ptr_->id  = img_right->header.seq;
  frame_left_ptr_-> time_stamp  = img_left->header.stamp.nsec;
  frame_right_ptr_->time_stamp  = img_right->header.stamp.nsec;
}

void
StereoFrame::rectifyImgs()
{
#ifdef USE_GPU
  cuda_rect_src.upload(frame_left_ptr_->getImg());
  cuda::remap(cuda_rect_src, cuda_rectified_img_left_,
              cuda_rectified_mapping_[0][0],
              cuda_rectified_mapping_[0][1],
              INTER_LINEAR);
  cuda_rectified_img_left_.download(rectified_img_left_);

  cuda_rect_src.upload(frame_right_ptr_->getImg());
  cuda::remap(cuda_rect_src, cuda_rectified_img_right_,
              cuda_rectified_mapping_[1][0],
              cuda_rectified_mapping_[1][1],
              INTER_LINEAR);
  cuda_rectified_img_right_.download(rectified_img_right_);
#else
  remap(frame_left_ptr_->getImg(), rectified_img_left_,
            rectified_mapping_[0][0], rectified_mapping_[0][1], INTER_LINEAR);
  remap(frame_right_ptr_->getImg(), rectified_img_right_,
            rectified_mapping_[1][0], rectified_mapping_[1][1], INTER_LINEAR);
#endif
}

void
StereoFrame::computeDisparityMap()
{

#ifdef USE_GPU
  cuda::GpuMat cuda_disp_left;

  // GPU implementation of stereoBM outputs uint8_t, i.e. CV_8U
  block_matcher_->compute(cuda_rectified_img_left_.clone(),
                          cuda_rectified_img_right_.clone(),
                          cuda_disp_left);

  cuda_disp_left.download(raw_disparity_map_);

  raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 1);

  // convert it from CV_8U to CV_16U for unified
  // calculation in filterDisparityMap() & unprojectPtCloud()
  raw_disparity_map_.convertTo(raw_disparity_map_, CV_16S, 16);
#else
  // CPU implementation of stereoBM outputs short int, i.e. CV_16S
  block_matcher_->compute(rectified_img_left_, rectified_img_right_, raw_disparity_map_);

  raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 0.0625);
#endif

}

void
StereoFrame::filterDisparityMap()
{
#ifdef USE_OPEN_CV_CONTRIB
  right_matcher_->compute(rectified_img_right_, rectified_img_left_, raw_right_disparity_map_);

  // Only takes CV_16S type cv::Mat
  wls_filter_->filter(raw_disparity_map_,
                      rectified_img_left_,
                      filtered_disparity_map_,
                      raw_right_disparity_map_);

  filtered_disparity_map_.convertTo(filtered_disparity_map_8u_, CV_8UC1, 0.0625);
#endif
}

void
StereoFrame::unprojectPtCloud()
{
  mat_vec3_pt_ = Mat_<Vec3f>(VGA_HEIGHT, VGA_WIDTH, Vec3f(0, 0, 0));

  for(int v = border_size_; v < trunc_img_height_end_; ++v)
  {
    for(int u = border_size_; u < trunc_img_width_end_; ++u)
    {
      Vec3f &point = mat_vec3_pt_.at<Vec3f>(v, u);

#ifdef USE_OPEN_CV_CONTRIB
      float disparity = (float)(filtered_disparity_map_.at<short int>(v, u)*0.0625);
#else
      float disparity = (float)(raw_disparity_map_.at<short int>(v, u)*0.0625);
#endif

      // do not consider pts that are farther than 8.6m, i.e. disparity < 6
      if(disparity >= 6)
      {
        point[2] = baseline_x_fx_/disparity;
        point[0] = (u-principal_x_)*point[2]/fx_;
        point[1] = (v-principal_y_)*point[2]/fy_;
      }
      color_buffer_[v*VGA_WIDTH+u] = rectified_img_left_.at<uint8_t>(v, u);
    }
  }

  color_mat_ = cv::Mat(VGA_HEIGHT, VGA_WIDTH, CV_8UC1, &color_buffer_[0]).clone();

  // @note Unfortunately, calling this WCloud constructor costs about the same amount
  // of time as we go through each pixel and unproject the pt cloud. Because there's
  // another nested for-loop inside WCloud implementation
  // Ideally these can be done in one shot but it involves changing openCV implementation
  // TODO maybe opencv projectPoints() is a good alternative
  pt_cloud_ = viz::WCloud(mat_vec3_pt_, color_mat_);
}

void
StereoFrame::unprojectROSPtCloud()
{
  int pt_cloud_count = 0;

  for(int v = border_size_; v < trunc_img_height_end_; ++v)
  {
    for(int u = border_size_; u < trunc_img_width_end_; ++u)
    {
#ifdef USE_OPEN_CV_CONTRIB
      float disparity = (float)(filtered_disparity_map_.at<short int>(v, u)*0.0625);
#else
      float disparity = (float)(raw_disparity_map_.at<short int>(v, u)*0.0625);
#endif

      **z_it_ = baseline_x_fx_/disparity;
      **x_it_ = (u-principal_x_)*(**z_it_)/fx_;
      **y_it_ = (v-principal_y_)*(**z_it_)/fy_;
      **r_it_ = **g_it_ = **b_it_ = rectified_img_left_.at<uint8_t>(v, u);

      ++(*x_it_); ++(*y_it_); ++(*z_it_);
      ++(*r_it_); ++(*g_it_); ++(*b_it_);

      ++pt_cloud_count;
    }
  }

  *x_it_ += -pt_cloud_count;
  *y_it_ += -pt_cloud_count;
  *z_it_ += -pt_cloud_count;
  *r_it_ += -pt_cloud_count;
  *g_it_ += -pt_cloud_count;
  *b_it_ += -pt_cloud_count;
}

#ifdef USE_DARKNET_ROS
void
StereoFrame::calcObjectInfo(const darknet_ros_msgs::BoundingBoxesConstPtr &b_box,
                            visualization_msgs::MarkerArray &marker_array)
{
  ROS_INFO("Got %d detections", (int)b_box->boundingBoxes.size());

  marker_array.markers.clear();
  marker_array.markers.resize(b_box->boundingBoxes.size());

  for (int i = 0; i < b_box->boundingBoxes.size(); ++i) {
    visualization_msgs::Marker marker = marker_template_;


    /**
     * Note that the bounding boxes were detected on original image
     * The point cloud and disparity map are rectified.
     * Here we use the center of the box to calculate
     * the "approximate" depth information.
     * For different applications and different objects, calculation varies.
     * Please make according changes to fit the need of your application.
     */

    cv::Rect roi;
    roi.x = b_box->boundingBoxes[i].xmin;
    roi.y = b_box->boundingBoxes[i].ymin;
    roi.width = b_box->boundingBoxes[i].xmax - b_box->boundingBoxes[i].xmin;
    roi.height = b_box->boundingBoxes[i].ymax - b_box->boundingBoxes[i].ymin;

    float v = roi.y + roi.height*0.5;
    float u = roi.x + roi.width*0.5;
    float disparity = (float)(filtered_disparity_map_.at<short int>(v, u)*0.0625);
    float dist_x, dist_y, dist_z;
    dist_z = baseline_x_fx_/disparity;
    dist_x = (u-principal_x_)*(dist_z)/fx_;
    dist_y = (v-principal_y_)*(dist_z)/fy_;
    float distance = sqrt(dist_z*dist_z + dist_y*dist_y + dist_x*dist_x);

    std::stringstream stream_x, stream_y, stream_z, stream_dist;
    stream_x << std::fixed << std::setprecision(2) << dist_x;
    std::string dist_x_str = stream_x.str();
    stream_y << std::fixed << std::setprecision(2) << dist_y;
    std::string dist_y_str = stream_y.str();
    stream_z << std::fixed << std::setprecision(2) << dist_z;
    std::string dist_z_str = stream_z.str();
    stream_dist << std::fixed << std::setprecision(2) << distance;
    std::string distance_str = stream_dist.str();

    marker.text = b_box->boundingBoxes[i].Class + "  " + distance_str + "m\n\tx:" + dist_x_str + "\n\ty:" + dist_y_str + "\n\tz:" + dist_z_str;
    marker.pose.position.x = dist_x;
    marker.pose.position.y = dist_y;
    marker.pose.position.z = dist_z-1; // -1 to avoid point cloud and text occlusion
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker_array.markers[i] = std::move(marker);
  }
}
#endif