/*
 * bag_logger.h
 *
 */

#ifndef BAG_LOGGER_H_
#define BAG_LOGGER_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>

#define DEFAULT_BAG_DIR "/home/ubuntu/testdata/"

#define LOG_MSG(topic, msg, level) { LOG_MSG_STAMP(topic, msg, ros::Time::now(), level) }

#define LOG_MSG_STAMP(topic, msg, t, level) { if (BagLogger::instance()->isLogging() && \
                                                 (level) <= BagLogger::instance()->getLogLevel()) { \
    BagLogger::instance()->bag.write((topic), (t), (msg)); } }

class BagLogger {
    static BagLogger *s_instance_;

    unsigned log_level_ {0};

    bool is_logging_ {false};

    std::string file_name_;

    std::string log_name_prefix_ {""};

    std::string log_name_seq_ {"0000"};

public:
    BagLogger() {};

    ~BagLogger() {};

    rosbag::Bag bag;

    static BagLogger *instance() {
        if (!s_instance_)
        {
            s_instance_ = new BagLogger;
        }
        return s_instance_;
    }

    bool isLogging() { return is_logging_; }

    std::string getLogFileDir() {
        using namespace std;
        using namespace boost::filesystem;

        path p("/media/ubuntu");
        for (auto i = directory_iterator(p); i != directory_iterator(); i++)
        {
            if (is_directory(i->path()))
            {
                std::size_t found = i->path().filename().string().find("LOG");
                if (found!=std::string::npos)
                {
//                cout << "FOUND: "+i->path().filename().string()+"  AT: "+i->path().string() << endl;
                    return i->path().string()+"/";
                }
            }
            else
                continue;
        }
        ROS_INFO("LG: NO LOG_XXX SD CARD FOUND");

        return string(DEFAULT_BAG_DIR);
    }

    std::string getSequence(std::string dir, std::string prefix) {
        using namespace std;
        using namespace boost::filesystem;
        using namespace boost;

        unsigned int seq_num = 0;

        path p(dir);
        for (auto i = directory_iterator(p); i != directory_iterator(); i++)
        {
            if (!is_directory(i->path())) //we eliminate directories
            {
//                cout << i->path().filename().string() << endl;
                std::vector<std::string> fields;
                boost::split(fields, i->path().filename().string(), boost::is_any_of("-"));
                if (fields.size() >= 2)
                {
                    if (fields[0] == prefix)
                    {
                        unsigned long tmp = atol(fields[1].c_str());
                        if (tmp < 9999 && tmp > seq_num)
                        {
                            seq_num = tmp;
                        }
                    }
                }
            }
            else
                continue;
        }

        seq_num++;

        std::stringstream s;
        s << std::setfill('0') << std::setw(4) << seq_num;

        log_name_seq_ = s.str();

        return log_name_seq_;
    }

    std::string getLogFileName(std::string prefix) {
        log_name_prefix_ = prefix;
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        std::string dir_str = getLogFileDir();

        std::string seq_str = getSequence(dir_str, prefix);

        std::string format_str = dir_str+prefix+"-"+seq_str+"-%Y-%m-%d-%H-%M-%S.bag";

        strftime(buffer, 80, format_str.c_str(), timeinfo);
        std::string str(buffer);

        return str;
    }

    void startLogging(std::string prefix, unsigned log_level) {
        if (is_logging_)
        {
            ROS_INFO("LG: Closing bag file %s", file_name_.c_str());
            bag.close();
            log_level_ = 0;
            is_logging_ = false;
        }

        if (log_level > 0)
        {
            file_name_ = getLogFileName(prefix);
            bag.open(file_name_, rosbag::bagmode::Write);
            ROS_INFO("Opening bag file %s", file_name_.c_str());
            log_level_ = log_level;
            is_logging_ = true;
        }
    }

    void stopLogging() {
        if (is_logging_)
        {
            log_level_ = 0;
            is_logging_ = false;
            ROS_INFO("LG: Closing bag file %s", file_name_.c_str());
            bag.close();
        }
        return;
    }
    unsigned getLogLevel() const {
        return log_level_;
    }

    bool isLogging() const {
        return is_logging_;
    }
};

#endif /* BAG_LOGGER_H_ */
