#ifndef ONBOARDSDK_M300_STEREO_PARAM_TOOL_H
#define ONBOARDSDK_M300_STEREO_PARAM_TOOL_H

#include "dji_vehicle.hpp"
#include "dji_perception.hpp"
#include "dji_osdk_ros/stereo_utility/config.hpp"
#include <sensor_msgs/CameraInfo.h>

using namespace M210_STEREO;
using namespace DJI;
using namespace DJI::OSDK;

class M300StereoParamTool
{
public:
  M300StereoParamTool(Vehicle *vehicle);
  ~M300StereoParamTool();

public:
  typedef struct DoubleCamParamType {
    Perception::DirectionType direction;
    double leftIntrinsics[9];
    double rightIntrinsics[9];
    double rotaionLeftInRight[9];
    double translationLeftInRight[3];
  } DoubleCamParamType;
	
	
  /*!*/
  Perception::CamParamType getM300stereoParams(Perception::DirectionType direction);
  
  void getM300stereoCameraInfo(Perception::CamParamType param, sensor_msgs::CameraInfo &left,
  sensor_msgs::CameraInfo &right);

  bool createStereoParamsYamlFile(std::string fileName,
                                  Perception::CamParamType param);

  void setParamFileForM300(std::string fileName);

 private:
  static void PerceptionCamParamCB(Perception::CamParamPacketType pack,
                                   void *userData);
  Vehicle *vehicle;
};


#endif //ONBOARDSDK_M300_STEREO_PARAM_TOOL_H
