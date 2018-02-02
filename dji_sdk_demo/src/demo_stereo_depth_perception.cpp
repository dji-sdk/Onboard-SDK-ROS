#include "dji_sdk_demo/demo_stereo_depth_perception.h"

using namespace M210_STEREO;

// for visualization purpose
bool is_disp_filterd;
bool vga_imgs_subscribed = false;

dji_sdk::StereoVGASubscription subscription;
ros::Publisher rect_img_left_publisher;
ros::Publisher rect_img_right_publisher;
ros::Publisher left_disparity_publisher;
ros::Publisher point_cloud_publisher;

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_advanced_sensing_depth_perception");
  ros::NodeHandle nh;

  if(argc >= 2){
    ROS_INFO("Input yaml file: %s", argv[1]);
  } else{
    ROS_ERROR("Please specify a yaml file with camera parameters");
    ROS_ERROR("rosrun dji_sdk_demo demo_stereo_depth_perception m210_stereo_param.yaml");
    return -1;
  }

  std::string yaml_file_path = argv[1];
  Config::setParamFile(yaml_file_path);

  //! Instantiate some relevant objects
  CameraParam::Ptr camera_left_ptr;
  CameraParam::Ptr camera_right_ptr;
  StereoFrame::Ptr stereo_frame_ptr;

  message_filters::Subscriber<sensor_msgs::Image> img_left_sub;
  message_filters::Subscriber<sensor_msgs::Image> img_right_sub;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *topic_synchronizer;


  //! Setup stereo frame
  camera_left_ptr   = CameraParam::createCameraParam(CameraParam::FRONT_LEFT);
  camera_right_ptr  = CameraParam::createCameraParam(CameraParam::FRONT_RIGHT);

  stereo_frame_ptr = StereoFrame::createStereoFrame(camera_left_ptr, camera_right_ptr);


  //! Setup ros related stuff
  rect_img_left_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_left_image", 10);
  rect_img_right_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/rectified_vga_front_right_image", 10);
  left_disparity_publisher =
    nh.advertise<sensor_msgs::Image>("/stereo_depth_perception/disparity_front_left_image", 10);
  point_cloud_publisher =
    nh.advertise<sensor_msgs::PointCloud2>("/stereo_depth_perception/unprojected_pt_cloud", 10);

  img_left_sub. subscribe(nh, "/dji_sdk/stereo_vga_front_left_images", 1);
  img_right_sub.subscribe(nh, "/dji_sdk/stereo_vga_front_right_images", 1);

  topic_synchronizer = new message_filters::TimeSynchronizer
    <sensor_msgs::Image, sensor_msgs::Image>(img_left_sub, img_right_sub, 10);


  //! For signal handling, e.g. if user terminate the program with Ctrl+C
  //! this program will unsubscribe the image stream if it's subscribed
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = shutDownHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);


  //! Display interactive prompt
  std::cout
    << std::endl
    << " AdvancedSensing API works in subscription mechanism              \n"
    << " Please remember to unsubscribe when the program terminates       \n"
    << " If any error messages occur, please reboot the aircraft          \n"
    << std::endl
    << "| Available commands:                                            |\n"
    << "| [a] Display rectified stereo images                            |\n"
    << "| [b] Display disparity map                                      |\n"
    << "| [c] Display filtered disparity map                             |\n"
    << "| [d] Display point cloud                                        |\n"
    << "| [e] Unsubscribe to VGA front stereo images                     |\n"
    << std::endl;
  char inputChar = ' ';
  std::cin >> inputChar;


  //! Subscribe to VGA images
  subscription.request.vga_freq = subscription.request.VGA_20_HZ;
  subscription.request.front_vga = 1;
  subscription.request.unsubscribe_vga = 0;

  if(!imgSubscriptionHelper(subscription)){
    return -1;
  }
  sleep(1);

  switch (inputChar)
  {
    case 'a': {
      topic_synchronizer->registerCallback(boost::bind(&displayStereoRectImgCallback,
                                                       _1, _2, stereo_frame_ptr));
    }
      break;
    case 'b': {
      topic_synchronizer->registerCallback(boost::bind(&displayStereoDisparityCallback,
                                                       _1, _2, stereo_frame_ptr));
    }
      break;
    case 'c': {
      topic_synchronizer->registerCallback(boost::bind(&displayStereoFilteredDisparityCallback,
                                                       _1, _2, stereo_frame_ptr));
    }
      break;
    case 'd': {
      topic_synchronizer->registerCallback(boost::bind(&displayStereoPtCloudCallback,
                                                       _1, _2, stereo_frame_ptr));
    }
      break;
    case 'e': {
      subscription.request.unsubscribe_vga = 1;
      if(!imgSubscriptionHelper(subscription)){
        return -1;
      }
      return 0;
    }
      break;
    default: {
      ROS_ERROR("Unknown input");
      subscription.request.unsubscribe_vga = 1;
      imgSubscriptionHelper(subscription);
      return 0;
    }
  }

  ros::spin();
}


void displayStereoRectImgCallback(const sensor_msgs::ImageConstPtr &img_left,
                                  const sensor_msgs::ImageConstPtr &img_right,
                                  StereoFrame::Ptr stereo_frame_ptr)
{
  //! Read raw images
  stereo_frame_ptr->readStereoImgs(img_left, img_right);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  visualizeRectImgHelper(stereo_frame_ptr);

  cv::waitKey(1);

  sensor_msgs::Image rect_left_img  = *img_left;
  sensor_msgs::Image rect_right_img = *img_right;
  memcpy((char*)(&rect_left_img.data[0]),
         stereo_frame_ptr->getRectLeftImg().data,
         img_left->height*img_left->width);
  memcpy((char*)(&rect_right_img.data[0]),
         stereo_frame_ptr->getRectRightImg().data,
         img_right->height*img_right->width);

  rect_img_left_publisher.publish(rect_left_img);
  rect_img_right_publisher.publish(rect_right_img);


  duration rectify_time_diff = rectify_end - rectify_start;
  ROS_INFO("This stereo frame takes %.2f ms to rectify", rectify_time_diff.count()*1000.0);
}

void displayStereoDisparityCallback(const sensor_msgs::ImageConstPtr &img_left,
                                    const sensor_msgs::ImageConstPtr &img_right,
                                    StereoFrame::Ptr stereo_frame_ptr)
{
  //! Read raw images
  stereo_frame_ptr->readStereoImgs(img_left, img_right);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  visualizeRectImgHelper(stereo_frame_ptr);

  visualizeDisparityMapHelper(stereo_frame_ptr);

  cv::waitKey(1);

  sensor_msgs::Image rect_left_img  = *img_left;
  sensor_msgs::Image rect_right_img = *img_right;
  sensor_msgs::Image disparity_map  = *img_left;
  memcpy((char*)(&rect_left_img.data[0]),
         stereo_frame_ptr->getRectLeftImg().data,
         img_left->height*img_left->width);
  memcpy((char*)(&rect_right_img.data[0]),
         stereo_frame_ptr->getRectRightImg().data,
         img_right->height*img_right->width);
  memcpy((char*)(&disparity_map.data[0]),
         stereo_frame_ptr->getDisparityMap().data,
         img_left->height*img_left->width);

  rect_img_left_publisher.publish(rect_left_img);
  rect_img_right_publisher.publish(rect_right_img);
  left_disparity_publisher.publish(disparity_map);


  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  ROS_INFO("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0);
}

void displayStereoFilteredDisparityCallback(const sensor_msgs::ImageConstPtr &img_left,
                                            const sensor_msgs::ImageConstPtr &img_right,
                                            StereoFrame::Ptr stereo_frame_ptr)
{
#ifdef USE_OPEN_CV_CONTRIB

  //! Read raw images
  stereo_frame_ptr->readStereoImgs(img_left, img_right);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  //! Filter disparity map
  timer filter_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->filterDisparityMap();
  is_disp_filterd = true;
  timer filter_end  = std::chrono::high_resolution_clock::now();

  visualizeRectImgHelper(stereo_frame_ptr);

  visualizeDisparityMapHelper(stereo_frame_ptr);

  sensor_msgs::Image rect_left_img  = *img_left;
  sensor_msgs::Image rect_right_img = *img_right;
  sensor_msgs::Image disparity_map  = *img_left;
  memcpy((char*)(&rect_left_img.data[0]),
         stereo_frame_ptr->getRectLeftImg().data,
         img_left->height*img_left->width);
  memcpy((char*)(&rect_right_img.data[0]),
         stereo_frame_ptr->getRectRightImg().data,
         img_right->height*img_right->width);
  memcpy((char*)(&disparity_map.data[0]),
         stereo_frame_ptr->getFilteredDispMap().data,
         img_left->height*img_left->width);

  rect_img_left_publisher.publish(rect_left_img);
  rect_img_right_publisher.publish(rect_right_img);
  left_disparity_publisher.publish(disparity_map);


  cv::waitKey(1);

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  duration filter_diff = filter_end - filter_start;
  ROS_INFO("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
            "%.2f ms to filter",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0,
          filter_diff.count()*1000.0);
#else
  ROS_INFO("openCV contrib is not enabled in CMakeLists. It's required for disparity map filtering");
#endif

}

void displayStereoPtCloudCallback(const sensor_msgs::ImageConstPtr &img_left,
                                  const sensor_msgs::ImageConstPtr &img_right,
                                  StereoFrame::Ptr stereo_frame_ptr)
{
  //! Read raw images
  stereo_frame_ptr->readStereoImgs(img_left, img_right);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  //! Filter disparity map
  timer filter_start= std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->filterDisparityMap();
  is_disp_filterd = true;
  timer filter_end  = std::chrono::high_resolution_clock::now();

  //! Unproject image to 3D point cloud
  timer pt_cloud_start= std::chrono::high_resolution_clock::now();
  stereo_frame_ptr->unprojectPtCloud();
  stereo_frame_ptr->unprojectROSPtCloud();
  timer pt_cloud_end  = std::chrono::high_resolution_clock::now();

  cv::viz::WCloud pt_cloud = stereo_frame_ptr->getPtCloud();
  PointCloudViewer::showPointCloud(pt_cloud);
  PointCloudViewer::spinOnce();

  visualizeRectImgHelper(stereo_frame_ptr);

  visualizeDisparityMapHelper(stereo_frame_ptr);

  cv::waitKey(1);

  sensor_msgs::Image rect_left_img  = *img_left;
  sensor_msgs::Image rect_right_img = *img_right;
  sensor_msgs::Image disparity_map  = *img_left;
  memcpy((char*)(&rect_left_img.data[0]),
         stereo_frame_ptr->getRectLeftImg().data,
         img_left->height*img_left->width);
  memcpy((char*)(&rect_right_img.data[0]),
         stereo_frame_ptr->getRectRightImg().data,
         img_right->height*img_right->width);
#ifdef USE_OPEN_CV_CONTRIB
  memcpy((char*)(&disparity_map.data[0]),
         stereo_frame_ptr->getFilteredDispMap().data,
         img_left->height*img_left->width);
#else
  memcpy((char*)(&disparity_map.data[0]),
         stereo_frame_ptr->getDisparityMap().data,
         img_left->height*img_left->width);
#endif

  rect_img_left_publisher.publish(rect_left_img);
  rect_img_right_publisher.publish(rect_right_img);
  left_disparity_publisher.publish(disparity_map);
  point_cloud_publisher.publish(stereo_frame_ptr->getROSPtCloud());

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  duration filter_diff = filter_end - filter_start;
  duration pt_cloud_diff = pt_cloud_end - pt_cloud_start;
  ROS_INFO("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
            "%.2f ms to filter, %.2f ms to unproject point cloud",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0,
          filter_diff.count()*1000.0,
          pt_cloud_diff.count()*1000.0);
}

void
visualizeRectImgHelper(StereoFrame::Ptr stereo_frame_ptr)
{
  cv::Mat img_to_show;

  cv::hconcat(stereo_frame_ptr->getRectLeftImg(),
              stereo_frame_ptr->getRectRightImg(),
              img_to_show);

  cv::resize(img_to_show, img_to_show,
             cv::Size(M210_STEREO::VGA_WIDTH*2, M210_STEREO::VGA_HEIGHT),
             (0, 0), (0, 0), cv::INTER_LINEAR);

  // draw epipolar lines to visualize rectification
  for(int j = 0; j < img_to_show.rows; j += 24 ){
    line(img_to_show, cv::Point(0, j),
         cv::Point(img_to_show.cols, j),
         cv::Scalar(255, 0, 0, 255), 1, 8);
  }

  cv::imshow("Rectified Stereo Imgs with epipolar lines", img_to_show);
}

void
visualizeDisparityMapHelper(StereoFrame::Ptr stereo_frame_ptr)
{
  cv::Mat raw_disp_map;
#ifdef USE_OPEN_CV_CONTRIB
  if(is_disp_filterd) {
    raw_disp_map = stereo_frame_ptr->getFilteredDispMap().clone();
  } else {
    raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
  }
#else
  raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
#endif

  double min_val, max_val;
  cv::minMaxLoc(raw_disp_map, &min_val, &max_val, NULL, NULL);

  cv::Mat scaled_disp_map;
  raw_disp_map.convertTo(scaled_disp_map, CV_8U, 255/(max_val-min_val), -min_val/(max_val-min_val));

  cv::imshow("Scaled disparity map", scaled_disp_map);
}

bool
imgSubscriptionHelper(dji_sdk::StereoVGASubscription &service)
{
  std::string action;
  if(service.request.unsubscribe_vga){
    action = "unsubscribed";
  }else{
    action = "subscribed";
  }

  if (ros::service::call("/dji_sdk/stereo_vga_subscription", service))
  {
    ROS_INFO("Successfully %s to VGA images", action.c_str());
    if(service.request.unsubscribe_vga){
      vga_imgs_subscribed = false;
    }else{
      vga_imgs_subscribed = true;
    }
  }
  else
  {
    ROS_ERROR("Failed to %s to VGA images", action.c_str());
    return false;
  }

  return true;
}

void
shutDownHandler(int s)
{
  ROS_INFO("Caught signal %d", s);

  if(vga_imgs_subscribed)
  {
    cv::destroyAllWindows();
    sleep(2);
    subscription.request.unsubscribe_vga = 1;
    imgSubscriptionHelper(subscription);
  }

  exit(1);
}