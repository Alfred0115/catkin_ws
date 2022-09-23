#include "base.h"
#include "signal/backtrace.hpp"
#include <plog/Log.h>
#include <plog/Record.h>
#include <iomanip>
#include <plog/Util.h>
#include <iomanip>
#include <plog/Initializers/RollingFileInitializer.h>
#include "BlockingCollection.h"
using namespace plog;
using namespace code_machina;
namespace plog
{
    class MyFormatter
    {
    public:
        static util::nstring header() // This method returns a header for a new file. In our case it is empty.
        {
            return util::nstring();
        }
         static util::nstring format(const Record& record) // This method returns a string from a record.
        {
            tm t;
            util::localtime_s(&t, &record.getTime().time);
            util::nostringstream ss;
                       
	    ss << record.getMessage() << PLOG_NSTR("\n");

            return ss.str();
        }
    };
}
namespace zjrobot_ns
{
    void signalHandle(int sigNum)
    {
      
       if(sigNum == SIGPIPE)
       {
          std::cout<<("signal pipe raised, now ignore !!!")<<std::endl;
       }
       else if(sigNum == SIGCHLD)
       {
	       //std::cout<<("signal SIGCHLD, now ignore !!!")<<std::endl;
       }
       else if(sigNum == SIGTERM)
       {
	       std::cout<<("signal SIGTERM, now ignore !!!")<<std::endl;
       }
       else
       {
         //robotOK = false;
         std::stringstream ss;
         tm t;
         Record tempRe(Severity::none," ", 0, " ", " ", 0);
         util::localtime_s(&t, &tempRe.getTime().time);
         ss << t.tm_year + 1900 << "-" << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mon + 1 << PLOG_NSTR("-") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mday << PLOG_NSTR(" ");
         ss << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_hour << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_min << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_sec << PLOG_NSTR(".") << std::setfill(PLOG_NSTR('0')) << std::setw(3) << static_cast<int> (tempRe.getTime().millitm) << PLOG_NSTR(" ");
         ss << PLOG_NSTR("[") << std::this_thread::get_id()<< PLOG_NSTR("] ");
         ss << PLOG_NSTR("[") << "serials"<< PLOG_NSTR("] ");
         std::cout<<ss.str()<<std::endl;
         addLogData(ss.str());
         std::stringstream ss1;
         ss1<<(strFormat("get signal %d, now abort !!! ", sigNum))<<std::endl;
         ss1<<"************************backtrace start******************"<<std::endl;
         ss1<<backtrace_ns::stacktrace{}<<std::endl;
         ss1<<"************************backtrace end******************"<<std::endl;
         addLogData(ss1.str());
         std::cout<<ss1.str()<<std::endl;
         raise(sigNum);
         robotOK = false;
         exit(-1);
       }
    }
void initSignalHandle()
    {
       signal(SIGPIPE, signalHandle);
       signal(SIGINT, signalHandle);
       signal(SIGQUIT, signalHandle);
       signal(SIGILL, signalHandle);
       signal(SIGTRAP, signalHandle);
       signal(SIGABRT, signalHandle);
       signal(SIGBUS, signalHandle);
       signal(SIGFPE, signalHandle);
       signal(SIGKILL, signalHandle);
       signal(SIGUSR1, signalHandle);
       signal(SIGSEGV, signalHandle);
       signal(SIGUSR2, signalHandle);
       signal(SIGALRM, signalHandle);
       signal(SIGTERM, signalHandle);
       signal(SIGSTKFLT, signalHandle);
       signal(SIGCLD, signalHandle);
       signal(SIGCHLD, signalHandle);
       signal(SIGCONT, signalHandle);
       signal(SIGSTOP, signalHandle);
       signal(SIGTSTP, signalHandle);
       signal(SIGTTIN, signalHandle);
       signal(SIGURG, signalHandle);
       signal(SIGXCPU, signalHandle);
       signal(SIGXFSZ, signalHandle);
       signal(SIGIO, signalHandle);
       signal(SIGWINCH, signalHandle);
       signal(SIGPROF, signalHandle);
    }

      code_machina::BlockingCollection<std::pair<std::string, int>> loggerDataColl(1000*100);
      std::shared_ptr<std::thread> loggerThread;
      std::mutex loggerMutex;
    
    void startLogger()
    {
      plog::init<plog::MyFormatter, LOG_FILE_SERIALS>(plog::debug,"debug_serials.log",1000*1000*100,10);
      loggerThread= std::shared_ptr<std::thread> (new std::thread(&loggerLoop));
	   loggerThread->detach();
    }
    void addLogData(const std::string &str, int fileLevel)
    {
       std::stringstream ss;
       std::pair<std::string, int> newLogger;
       Record tempRe(Severity::none," ", 0, " ", " ", 0);
       tm t;
       util::localtime_s(&t, &tempRe.getTime().time);
       ss << t.tm_year + 1900 << "-" << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mon + 1 << PLOG_NSTR("-") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_mday << PLOG_NSTR(" ");
       ss << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_hour << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_min << PLOG_NSTR(":") << std::setfill(PLOG_NSTR('0')) << std::setw(2) << t.tm_sec << PLOG_NSTR(".") << std::setfill(PLOG_NSTR('0')) << std::setw(3) << static_cast<int> (tempRe.getTime().millitm) << PLOG_NSTR(" ");
       ss << PLOG_NSTR("[") << std::this_thread::get_id()<< PLOG_NSTR("] ");
       ss << PLOG_NSTR("[") << "serials"<< PLOG_NSTR("] ");
       if(fileLevel != LOG_FILE_POSE)
       {
	      newLogger.first = ss.str() + str;
       }
       else
       {
         newLogger.first = str;
       }
       newLogger.second = fileLevel;
       std::lock_guard<std::mutex> lockG(loggerMutex);
       loggerDataColl.add(newLogger);
    }
    void loggerLoop()
    {
      std::pair<std::string, int> sliceData;
      BlockingCollectionStatus loggerStatus;
      while(robotOK)
      {
         loggerStatus = loggerDataColl.take(sliceData);
         if(loggerStatus!= BlockingCollectionStatus::Ok)
         {
            std::cout << "logger added  error!" << std::endl;
         }
         else
         {
            if(sliceData.second == LOG_FILE_SERIALS)
            {
               PLOG_(LOG_FILE_SERIALS, plog::debug)<<sliceData.first;
            }
         }
      }
    }
    std::shared_ptr<std::stringstream> getNewStringStreamPtr()
    {
      return std::shared_ptr<std::stringstream>(new std::stringstream ());
    }
}
