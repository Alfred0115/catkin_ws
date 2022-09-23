#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include "amr_msgs_srv/amr_info_status.h"

std::string start_voice_, errorStatus00_, errorStatus01_, errorStatus02_,
    warningStatus00_, warningStatus01_, warningStatus02_, warningStatus03_,
    warningStatus04_, warningStatus05_;

void playWav(std::string voice_dir)
{
    const std::string playPath = "play " + voice_dir;
    system(playPath.c_str());
}

void topicCallBack(const amr_msgs_srv::amr_info_status::ConstPtr &msg)
{
    if (msg->errorStatus == 0)
    {
        //playWav(errorStatus00_);
        if (msg->warningStatus == 0)
        {
            std::cout << "get topic" << std::endl;
            playWav(warningStatus00_);
        }
        else if (msg->warningStatus == 1)
        {
            playWav(warningStatus01_);
        }
        else if (msg->warningStatus == 2)
        {
            playWav(warningStatus02_);
        }
        else if (msg->warningStatus == 3)
        {
            playWav(warningStatus03_);
        }
        else if (msg->warningStatus == 4)
        {
            playWav(warningStatus04_);
        }
        else if (msg->warningStatus == 5)
        {
            playWav(warningStatus05_);
        }
    }
    else if (msg->errorStatus == 1)
    {
        playWav(errorStatus01_);
    }
    else if (msg->errorStatus == 2)
    {
        playWav(errorStatus02_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amr_play_voice_node");
    ros::NodeHandle nd("~");
    nd.param("start_voice", start_voice_, std::string(""));
    nd.param("errorStatus00", errorStatus00_, std::string(""));
    nd.param("errorStatus01", errorStatus01_, std::string(""));
    nd.param("errorStatus02", errorStatus02_, std::string(""));
    nd.param("warningStatus00", warningStatus00_, std::string(""));
    nd.param("warningStatus01", warningStatus01_, std::string(""));
    nd.param("warningStatus02", warningStatus02_, std::string(""));
    nd.param("warningStatus03", warningStatus03_, std::string(""));
    nd.param("warningStatus04", warningStatus04_, std::string(""));
    nd.param("warningStatus05", warningStatus05_, std::string(""));

    //初始化语音
    playWav(start_voice_);

    //std::cout << "start voice: " << start_voice_  << std::endl;

    ros::Subscriber sub = nd.subscribe("/amrCommunication/amrInfoStatus", 3, topicCallBack);
    ros::spin();

    return 0;
}