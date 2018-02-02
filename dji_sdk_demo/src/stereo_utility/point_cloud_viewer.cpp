#include "stereo_utility/point_cloud_viewer.hpp"

PointCloudViewer* PointCloudViewer::single_instance_ = new PointCloudViewer("Point Cloud Viewer");

PointCloudViewer::PointCloudViewer(const std::string &name)
  : viewer_(name)
  , view_pos_(-1.0, -2.0, -3.0)
  , view_point_(0, 0, 0)
  , view_y_dir_(0, 1, 0)
  , world_coordinate_(.3)
{
  cv::Affine3d view_pose = cv::viz::makeCameraPose(view_pos_,
                                                   view_point_,
                                                   view_y_dir_);
  viewer_.setViewerPose(view_pose);

  world_coordinate_.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);

  viewer_.showWidget("World", world_coordinate_);
  cv::Affine3d M = cv::Affine3d::Identity();
  viewer_.setWidgetPose("World", M );
}

PointCloudViewer::~PointCloudViewer()
{

}

PointCloudViewer&
PointCloudViewer::instance()
{
  return *PointCloudViewer::single_instance_;
}

PointCloudViewer*
PointCloudViewer::instancePtr()
{
  return PointCloudViewer::single_instance_;
}

void
PointCloudViewer::showPointCloud(cv::viz::WCloud &cloud)
{
  PointCloudViewer *this_ptr = PointCloudViewer::instancePtr();

  cloud.setRenderingProperty( cv::viz::POINT_SIZE, 3);
  this_ptr->viewer_.showWidget("Cloud", cloud);
}