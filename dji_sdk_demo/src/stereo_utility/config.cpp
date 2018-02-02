#include "stereo_utility/config.hpp"

using namespace M210_STEREO;

Config* Config::single_instance_ = NULL;

Config::Config()
{
}

Config::~Config()
{
  if (file_.isOpened())
  {
    file_.release();
  }
}

Config&
Config::instance()
{
  return *Config::single_instance_;
}

Config*
Config::instancePtr()
{
  return Config::single_instance_;
}

void
Config::setParamFile(const std::string& file_name)
{
  if(!Config::single_instance_)
  {
    Config::single_instance_ = new Config();
  }

  Config::instancePtr()->file_ = cv::FileStorage( file_name, cv::FileStorage::READ );

  if(!Config::instancePtr()->file_.isOpened())
  {
    std::cerr << "Failed to open " << file_name << " file\n";
    Config::instancePtr()->file_.release();
  }
}
