/** @file demo_camera_gimbal.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use camera and gimbal APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_camera_gimbal.h>

// global variables
geometry_msgs::Vector3Stamped gimbal_angle;
ros::Subscriber         gimbal_angle_subscriber;
ros::Publisher          gimbal_angle_cmd_publisher;
ros::Publisher          gimbal_speed_cmd_publisher;
ros::ServiceClient      drone_activation_service;
ros::ServiceClient      camera_action_service;

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sdk_demo_camera_gimbal");
  ros::NodeHandle nh;

  // ROS stuff
  gimbal_angle_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>
    ("dji_sdk/gimbal_angle", 10, &gimbalAngleCallback);
  gimbal_angle_cmd_publisher = nh.advertise<dji_sdk::Gimbal>
    ("dji_sdk/gimbal_angle_cmd", 10);
  gimbal_speed_cmd_publisher = nh.advertise<geometry_msgs::Vector3Stamped>
    ("dji_sdk/gimbal_speed_cmd", 10);
  drone_activation_service  = nh.serviceClient<dji_sdk::Activation>
    ("dji_sdk/activation");
  camera_action_service     = nh.serviceClient<dji_sdk::CameraAction>
    ("dji_sdk/camera_action");

  // Activate
  if (activate().result) {
    ROS_INFO("Activated successfully");
  } else {
    ROS_WARN("Failed activation");
    return -1;
  }

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Exercise gimbal and camera control                         |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch(inputChar) {
    case 'a':
      gimbalCameraControl();
      break;
    default:
      break;
  }

  ros::spin();

  return 0;
}

bool
gimbalCameraControl()
{
  int responseTimeout = 0;

  GimbalContainer               gimbal;
  RotationAngle                 initialAngle;
  RotationAngle                 currentAngle;
  geometry_msgs::Vector3Stamped gimbalSpeed;

  // Get Gimbal initial values
  ros::spinOnce();
  initialAngle.roll = gimbal_angle.vector.y;
  initialAngle.pitch = gimbal_angle.vector.x;
  initialAngle.yaw = gimbal_angle.vector.z;
  ROS_INFO("Please note that the gimbal yaw angle you see "
             "is w.r.t absolute North, and depends on your "
             "magnetometer calibration.");
  ROS_INFO("Initial Gimbal rotation angle: [ %f, %f, %f ] deg",
           initialAngle.roll,
           initialAngle.pitch,
           initialAngle.yaw);

  // Re-set Gimbal to initial values
  gimbal = GimbalContainer(0,0,0,2,1,initialAngle);
  doSetGimbalAngle(&gimbal);

  ROS_INFO("Setting new Gimbal rotation angle to [0,20,180] deg"
             "using incremental control:");
  // Get current gimbal data to calc precision error in post processing
  ros::spinOnce();
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;

  gimbal = GimbalContainer(0,20,180,2,0,initialAngle,currentAngle);
  doSetGimbalAngle(&gimbal);

  ros::spinOnce();
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;

  displayResult(&currentAngle);

  // Take picture
  ROS_INFO("Ensure SD card is present.");
  ROS_INFO("Taking picture..");
  if (takePicture()) {
    ROS_INFO("Picture taken, Check DJI GO App or SD card for a new picture.");
  }else{
    ROS_WARN("Failed taking picture");
  }

  ROS_INFO("Setting new Gimbal rotation angle to [0,-50, 0] deg"
             "using absolute control:");
  gimbal = GimbalContainer(0,-50,0,2,1,initialAngle);
  doSetGimbalAngle(&gimbal);


  ros::spinOnce();
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;
  displayResult(&currentAngle);

  // Start video: We will keep the video doing for the duration of the speed control.
  ROS_INFO("Ensure SD card is present.");
  ROS_INFO("Starting video..");
  if (startVideo()) {
    ROS_INFO("Start recording video");
  }else{
    ROS_WARN("Failed recording video");
    return false;
  }

  // Speed control
  ROS_INFO("Gimbal Speed Description: \n\n"
            "Roll  - unit rad/s input rate [-pi, pi]\n"
            "Pitch - unit rad/s input rate [-pi, pi]\n"
            "Yaw   - unit rad/s input rate [-pi, pi]\n\n");

  ROS_INFO("Setting Roll rate to 10 deg/s, Pitch rate to 5 deg/s, Yaw Rate to -20 deg/s.");
  gimbalSpeed.vector.y = DEG2RAD(10);
  gimbalSpeed.vector.x = DEG2RAD(5);
  gimbalSpeed.vector.z = DEG2RAD(-20);

  int speedControlDurationMs = 4000;
  int incrementMs = 100;
  for (int timer = 0; timer < speedControlDurationMs; timer+=incrementMs) {
    gimbal_speed_cmd_publisher.publish(gimbalSpeed);
    usleep(incrementMs*1000);
  }

  ros::spinOnce();
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;
  displayResult(&currentAngle);

  // Reset the position
  ROS_INFO("Resetting position...");
  gimbal = GimbalContainer(0,0,0,2,1,initialAngle);
  doSetGimbalAngle(&gimbal);

  ros::spinOnce();
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;
  displayResult(&currentAngle);

  // Stop the video
  ROS_INFO("Stopping video...");
  if (stopVideo()) {
    ROS_INFO("Stop recording video, Check DJI GO App or SD card for a new video.");
  }else{
    ROS_WARN("Failed stopping video");
    return false;
  }

  return true;
}

void
doSetGimbalAngle(GimbalContainer *gimbal)
{
  dji_sdk::Gimbal gimbal_angle_data;
  gimbal_angle_data.mode |= 0;
  gimbal_angle_data.mode |= gimbal->isAbsolute;
  gimbal_angle_data.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbal_angle_data.mode |= gimbal->roll_cmd_ignore << 2;
  gimbal_angle_data.mode |= gimbal->pitch_cmd_ignore << 3;
  gimbal_angle_data.ts    = gimbal->duration;
  gimbal_angle_data.roll  = DEG2RAD(gimbal->roll);
  gimbal_angle_data.pitch = DEG2RAD(gimbal->pitch);
  gimbal_angle_data.yaw   = DEG2RAD(gimbal->yaw);

  gimbal_angle_cmd_publisher.publish(gimbal_angle_data);
  // Give time for gimbal to sync
  sleep(4);
}

void
displayResult(RotationAngle *currentAngle)
{
  ROS_INFO("New Gimbal rotation angle is [ %f, %f, %f ] deg",
           currentAngle->roll,
           currentAngle->pitch,
           currentAngle->yaw);
}

ServiceAck
activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if(!activation.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return {activation.response.result, activation.response.cmd_set,
          activation.response.cmd_id, activation.response.ack_data};
}

bool
takePicture()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 0;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

bool
startVideo()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 1;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

bool
stopVideo()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 2;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

void
gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  gimbal_angle = *msg;
}