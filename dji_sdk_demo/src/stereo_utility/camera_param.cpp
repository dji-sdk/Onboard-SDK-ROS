#include "stereo_utility/camera_param.hpp"

using namespace M210_STEREO;

CameraParam::CameraParam(uint8_t location)
  : location_(location)
{
  if(!initIntrinsicParam())
  {
    std::cerr << "Failed to init intrinsic parameter";
  }

  if(!initDistortionParam())
  {
    std::cerr << "Failed to init distortion parameter";
  }
}

CameraParam::~CameraParam()
{

}

CameraParam::Ptr CameraParam::createCameraParam(uint8_t position)
{
  return std::make_shared<CameraParam>(position);
}

bool
CameraParam::initDistortionParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_distortion_ = Config::get<cv::Mat>("leftDistCoeffs");
  }
  else if(location_ == FRONT_RIGHT)
  {
    param_distortion_ = Config::get<cv::Mat>("rightDistCoeffs");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}

bool
CameraParam::initIntrinsicParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("leftCameraIntrinsicMatrix");
  }
  else if(location_ == FRONT_RIGHT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("rightCameraIntrinsicMatrix");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}
