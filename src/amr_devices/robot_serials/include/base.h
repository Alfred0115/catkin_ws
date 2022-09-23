

#ifndef _UTIL_BASE_H
#define _UTIL_BASE_H
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <deque>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <math.h>
#include <cmath>
#include <list>
#include <errno.h>
#include <linux/unistd.h>       /* for _syscallX macros/related stuff */
#include <linux/kernel.h>       /* for struct sysinfo */
#include <sys/sysinfo.h>

#include <functional>
#include <mutex>
//#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Int8.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"

#include "tf/transform_datatypes.h"

#include <iostream>
#include <fstream>

#include <cstdlib>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <signal.h>
//#include "yaml-cpp/yaml.h"
#include <fstream>
#include <actionlib_msgs/GoalID.h>
#include <jsoncpp/json/json.h>
static bool robotOK = true;

namespace zjrobot_ns
{  

    struct Pose2D
    {
      double x;
      double y;
      double theta;
      bool vaild;
      long long x86TimeStamp;
    };
    enum
    {
      LOG_FILE_POSE = 1,
      LOG_FILE_MOVE_BASE,
      LOG_FILE_TEB_LOCAL_PLANNER,
      LOG_FILE_BOARD,
      LOG_FILE_STATEMACHINE,
      LOG_FILE_SERIALS,
      LOG_FILE_GLOBAL_PLANNER,
      LOG_FILE_COSTMAP_2D
    };
    template<typename ... Args> 
    std::string strFormat(const std::string& format, Args ... args)
    { 
      size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...); 
      char bytes[size]; 
      snprintf(bytes, size, format.c_str(), args ...);
      return std::string(bytes);
    };
    template<typename ... Args>
    std::string str(const std::string &format, Args ... args)
    {
      size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...);
      char bytes[size];
      snprintf(bytes, size, format.c_str(), args ...);
    }
    template<typename T>
    T adder(T v)
    {
      return v;
    }
    template<typename T, typename ...Args>
    T adder(T frist, Args ... args)
    {
      return frist + adder(args...);
    }
   
    void initSignalHandle();
    void signalHandle(int sigNum);
    void startLogger();
    void addLogData(const std::string &str, int fileLevel = LOG_FILE_SERIALS);
    void loggerLoop();
    std::shared_ptr<std::stringstream> getNewStringStreamPtr();
    static uint64_t m_logDataSize = 0;
    template<typename ... Args> 
    void ROS_INFOMA(const std::string& format, Args ... args)
    {
      size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...); 
      char bytes[size]; 
      snprintf(bytes, size, format.c_str(), args ...);
      std::string sss(bytes);
      addLogData(sss);
    };
}

#endif //DATASERVER_BASE_H
