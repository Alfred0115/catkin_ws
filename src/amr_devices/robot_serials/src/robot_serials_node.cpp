#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <unistd.h>
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/Range.h>

#include <robot_msgs/robot_break_status.h>
#include <robot_msgs/robot_light_status.h>
#include <robot_msgs/set_stm32config_float.h>
#include <robot_msgs/robot_debug_msg.h>
#include "robot_serials.h"
#include "sensor_msgs/Imu.h" //lyy
#include <deque>
#include <geometry_msgs/Vector3.h>
#include "base.h"
using namespace zjrobot_ns;
using namespace std;
static float odom_x = 0.0f, odom_y = 0.0f, odom_th = 0.0f;           // lyy
static float odom_th_last = 0.0f, imu_th = 0.0f, imu_th_last = 0.0f; // lyy
static bool imu_th_init = false;                                     // lyy
static std::deque<float> sm_que, sm_que_backup;
static float sm_last_v = 0.0f, sm_last_w = 0.0f;
static float last_in_v = 0.0f, last_in_w = 0.0f; // lyy
geometry_msgs::Twist inMsg_m;
float v_out = 0.0f, w_out = 0.0f;

boost::asio::serial_port *psp_control = NULL;
boost::asio::serial_port *psp_config = NULL;
bool n_break_status = true;
bool n_charger_ctrl = false;
unsigned char n_sonar_ctrl = 0xFF;
float n_control_rates = 10;
unsigned short n_light_status = 0x0;
std::mutex lock_odom;
ros::Time m_last_recv_odom_time;
bool m_odom_recv_initd = false;
// lyy
void my_smoother_init()
{
    sm_que.clear();
    sm_que_backup.clear();
    float s_val = 0.0f;
    int cout = 2 * 3;
    // for(int i=-cout; i<cout; i+=1){
    //     s_val = sigmoid(i);// i: -10 ~ 10
    //     sm_que.push_back(s_val);
    //     sm_que_backup.push_back(s_val);
    // }
    for (int i = 0; i < cout; i += 1)
    {
        s_val += 1.0f / cout;
        sm_que.push_back(s_val);
        sm_que_backup.push_back(s_val);
    }
    sm_que.pop_back();
    sm_que_backup.pop_back();
    sm_que.push_back(1.0f);
    sm_que_backup.push_back(1.0f);
}

// lyy
void my_velocity_smoother(const float &v_in, const float &w_in, float &v_out, float &w_out)
{
    float new_in_detect_v = v_in - last_in_v;
    float new_in_detect_w = w_in - last_in_w;

    if ((fabs(new_in_detect_v) > 0.005) || (fabs(new_in_detect_w) > 0.005))
    { // new value arrives
        // if(sm_que.size()>0)
        { // not finish
            sm_last_v = v_out;
            sm_last_w = w_out;
        }
        sm_que = sm_que_backup;
    }

    if (sm_que.size() > 0)
    { // interpolation
        float delta_v = v_in - sm_last_v;
        float delta_w = w_in - sm_last_w;
        v_out = sm_last_v + delta_v * sm_que.front();
        w_out = sm_last_w + delta_w * sm_que.front();
        sm_que.pop_front();
    }
    else
    { // finish interpolation
        sm_last_v = last_in_v;
        sm_last_w = last_in_w;
    }

    last_in_v = v_in;
    last_in_w = w_in;
}

unsigned char cal_check_sum(unsigned char *pdata, int n_size)
{
    unsigned char tmp_sum = 0x0;

    for (int i = 0; i < n_size - 2; i++)
    {
        tmp_sum += *(unsigned char *)(pdata + i);
    }

    return tmp_sum;
}

bool error_frame_paser(Error_Frame *pError_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers, boost::asio::serial_port *psp)
{
    diagnostic_msgs::DiagnosticArray error_arr;
    error_arr.header.stamp = t_current_time;
    error_arr.header.frame_id = "stm32";

    diagnostic_msgs::DiagnosticStatus error_msg;
    error_msg.level = pError_Frame->value == 1 ? diagnostic_msgs::DiagnosticStatus::ERROR : diagnostic_msgs::DiagnosticStatus::OK;
    error_msg.message = "hardware_id is the error number";
    error_msg.hardware_id = to_string(pError_Frame->value);
    error_arr.status.push_back(error_msg);
    ros_handlers.pub_diag.publish(error_arr);
    ROS_INFOMA("error_frame_paser():error_msg.hardware_id:%s", error_msg.hardware_id.c_str());
    // write ack to serial port
    pError_Frame->header.type = 0x9;
    try
    {
        if (psp != NULL)
        {
            boost::system::error_code err;
            psp->write_some(boost::asio::buffer((unsigned char *)pError_Frame, sizeof(Error_Frame)), err);
            if (err)
            {
                psp->write_some(boost::asio::buffer((unsigned char *)pError_Frame, sizeof(Error_Frame)), err);
            }
        }
    }
    catch (...)
    {
        return false;
    }

    return true;
}

nav_msgs::Odometry odom_msg;

void init_odom_msg(Serial_Config &config)
{
    odom_msg.header.frame_id = config.str_odom_id;
    odom_msg.child_frame_id = config.str_robot_id;
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;

    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    odom_msg.pose.pose.orientation.w = 0;

    for (int i = 0; i < 36; i++)
    {
        odom_msg.pose.covariance[i] = 0.0;
    }

    odom_msg.pose.covariance[0] = 1e-3; // px   0.1 -> 1e-3
    odom_msg.pose.covariance[7] = 1e-3; // py  0.1 -> 1e-3
    odom_msg.pose.covariance[14] = 1e6; // pz
    odom_msg.pose.covariance[21] = 1e6; // ax
    odom_msg.pose.covariance[28] = 1e6; // ay
    odom_msg.pose.covariance[35] = 1e3; // az  1e9 -> 1e3

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    for (int i = 0; i < 36; i++)
    {
        odom_msg.twist.covariance[i] = 0.0;
    }

    odom_msg.twist.covariance[0] = 1e-9; // vx
    odom_msg.twist.covariance[7] = 1e-3; // vy
    odom_msg.twist.covariance[8] = 1e-9;
    odom_msg.twist.covariance[14] = 1e6;  // vz
    odom_msg.twist.covariance[21] = 1e6;  // wx
    odom_msg.twist.covariance[28] = 1e6;  // wy
    odom_msg.twist.covariance[35] = 1e-6; // wz

    my_smoother_init(); // lyy
}

bool odom_frame_paser(Odom_Frame *pOdom_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers)
{
    //printf("pub odom...\n");
    odom_msg.header.stamp = t_current_time;
    ROS_INFOMA("recv");
    if (fabs(pOdom_Frame->position_x) > 0 ||
        fabs(pOdom_Frame->position_y) > 0 ||
        fabs(pOdom_Frame->linear_vel) > 0 ||
        fabs(pOdom_Frame->angular_vel) > 0)
        ROS_INFOMA("odom_frame_paser:position_x:%f,position_y%f,linear_vel%f,angular_vel:%f,robot_state:0x %x,other_state:0x %x,\
    error status:motor l:%d,:r%d,urlter_wave_status:%d,",
                   pOdom_Frame->position_x, pOdom_Frame->position_y, pOdom_Frame->linear_vel, pOdom_Frame->angular_vel,
                   pOdom_Frame->robot_state, pOdom_Frame->other_state,
                   pOdom_Frame->err_code_l, pOdom_Frame->err_code_r, pOdom_Frame->urlter_wave_status);

    float d_left = pOdom_Frame->position_x;
    float d_right = pOdom_Frame->position_y;
    // odom_msg.pose.pose.position.x = pOdom_Frame->position_x;
    // odom_msg.pose.pose.position.y = pOdom_Frame->position_y;
    odom_msg.twist.twist.linear.x = pOdom_Frame->linear_vel;
    odom_msg.twist.twist.angular.z = pOdom_Frame->angular_vel;

    float Distance_Center = (d_left + d_right) / 2;
    float Theta_Center = (d_right - d_left) / 0.64f;

    if (Distance_Center != 0)
    {
        float Distance_X = cos(Theta_Center) * Distance_Center;
        float Distance_Y = sin(Theta_Center) * Distance_Center;
        odom_x += cos(odom_th) * Distance_X - sin(odom_th) * Distance_Y;
        odom_y += sin(odom_th) * Distance_X + cos(odom_th) * Distance_Y;
    }

    if (Theta_Center != 0)
    {
        odom_th_last = odom_th;
        odom_th += Theta_Center;
    }
    float ratio = 0.9f;
    float fusion_th = (1.0f - ratio) * odom_th + ratio * imu_th;

    tf::Matrix3x3 obs_mat;
    // obs_mat.setEulerYPR((float)pOdom_Frame->position_w, 0, 0);
    obs_mat.setEulerYPR((float)odom_th, 0, 0); // lyy

    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = fusion_th * 57.2957795f;
    odom_msg.pose.pose.orientation.x = q_tf.getX();
    odom_msg.pose.pose.orientation.y = q_tf.getY();
    odom_msg.pose.pose.orientation.z = q_tf.getZ();
    odom_msg.pose.pose.orientation.w = q_tf.getW();

    //std::cout << "odom_x: " << odom_x << ", odom_y: " << odom_y << std::endl;
 
    ros_handlers.pub_odom.publish(nav_msgs::Odometry(odom_msg));

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
    transform.setRotation(q_tf);
    // try {
    //	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
    // } catch (...) {
    //	    return false;
    // }
    robot_msgs::robot_break_status robot_break;
    robot_break.header.stamp = t_current_time;
    robot_break.header.frame_id = "robot_break_status";

    robot_break.control_break = pOdom_Frame->robot_state & 0x1;
    robot_break.sonar_break = pOdom_Frame->robot_state >> 1 & 0x1;
    robot_break.error_break = pOdom_Frame->robot_state >> 2 & 0x1;
    robot_break.button_break = pOdom_Frame->robot_state >> 3 & 0x1;
    ros_handlers.pub_break.publish(robot_break);

    std_msgs::UInt8 sonar_status;
    sonar_status.data = (pOdom_Frame->other_state & 0x00FF0000) >> 16;
    ros_handlers.pub_sonar_status.publish(sonar_status);

    std_msgs::Bool charger_status;
    charger_status.data = pOdom_Frame->robot_state >> 31 & 0x1;
    ros_handlers.pub_charger.publish(charger_status);

    robot_msgs::robot_light_status robot_light;
    robot_light.header.stamp = t_current_time;
    robot_light.header.frame_id = "robot_light_status";

    robot_light.light_pattern = pOdom_Frame->other_state & 0x0000FFFF;
    ros_handlers.pub_light.publish(robot_light);
    return true;
}
bool sonar_frame_paser(Sonar_Frame *pSonar_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers)
{
    ROS_INFOMA("sonar_frame_paser():index:%d,range0~5:%f,%f,%f,%f,%f,",
               pSonar_Frame->index, pSonar_Frame->range0, pSonar_Frame->range1, pSonar_Frame->range2,
               pSonar_Frame->range3, pSonar_Frame->range4, pSonar_Frame->range5);
    sensor_msgs::Range sonar_data;
    sonar_data.header.stamp = t_current_time;
    sonar_data.field_of_view = config.f_fov_sonar;
    sonar_data.min_range = config.f_min_sonar;
    sonar_data.max_range = config.f_max_sonar;
    for (int i = 0; i < 6; i++)
    {
        sprintf((char *)sonar_data.header.frame_id.c_str(), "amr_sonar_id_0%d", i);
        sonar_data.range = *(&pSonar_Frame->range0 + i);
        ros_handlers.pub_sonar.publish(sensor_msgs::Range(sonar_data));
    }

    return true;
}

bool volume_frame_paser(Volume_Frame *pVolume_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers)
{
    return true;
}

bool log_frame_paser(Log_Frame *pLog_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers)
{
    ROS_INFOMA("log_frame_paser():L_wv%f, R_wv:%f, linear_vel:%f, angular_vel:%f,\
    linear_acc:%f, angular_vel:%f, position_x:%f, position_y:%f,\
    position_w:%f ",
               pLog_Frame->L_wv, pLog_Frame->R_wv, pLog_Frame->linear_vel, pLog_Frame->angular_vel,
               pLog_Frame->linear_acc, pLog_Frame->angular_vel, pLog_Frame->position_x, pLog_Frame->position_y,
               pLog_Frame->position_w);

    robot_msgs::robot_debug_msg msg;
    msg.header.stamp = t_current_time;
    msg.header.frame_id = "stm32_debug";

    msg.L_wv = pLog_Frame->L_wv;
    msg.R_wv = pLog_Frame->R_wv;
    msg.linear_vel = pLog_Frame->linear_vel;
    msg.angular_vel = pLog_Frame->angular_vel;
    msg.linear_acc = pLog_Frame->linear_acc;
    msg.angular_vel = pLog_Frame->angular_vel;
    msg.position_x = pLog_Frame->position_x;
    msg.position_y = pLog_Frame->position_y;
    msg.position_w = pLog_Frame->position_w;

    ros_handlers.pub_log.publish(msg);

    return true;
}

bool config_ack_frame_paser(Config_Frame *pConfig_ACK_Frame, ros::Time t_current_time,
                            Serial_Config &config, Ros_Handlers &ros_handlers,
                            boost::timed_mutex &config_mutex, int &n_command_index, float &f_command_value)
{

    n_command_index = pConfig_ACK_Frame->index;
    f_command_value = pConfig_ACK_Frame->value;
    config_mutex.unlock();
    return true;
}

void serial_read_thread(Serial_Config &config, std::string &sz_dev_name, Ros_Handlers &ros_handlers,
                        boost::asio::serial_port *&psp, boost::timed_mutex &config_mutex, int &n_command_index, float &f_command_value)
{
    const int n_rx_buf_size = 128;
    unsigned char rx_buf[n_rx_buf_size];

    ros::Rate rate(1);
    boost::asio::io_service io_s;
    ros::Time t_current_time;

    int n_pkg_len = 0;
    int n_pkg_header_len = sizeof(Frame_header);

    while (ros::ok())
    {
        // seg connect the serial device
        try
        {
            if (psp == NULL)
            {
                // printf ("Try to connect device : %s\n", sz_dev_name.c_str());
                psp = new boost::asio::serial_port(io_s, sz_dev_name);
                if (NULL == psp)
                {
                    rate.sleep();
                    continue;
                }

                psp->set_option(boost::asio::serial_port::baud_rate(config.n_baud_rate));
                psp->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
                psp->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                psp->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                psp->set_option(boost::asio::serial_port::character_size(config.n_char_size));
            }
        }
        catch (...)
        {
            if (psp != NULL)
            {
                delete psp;
                psp == NULL;
            }
            rate.sleep();
            continue;
        }

        try
        {
            // seg read sync byte of header from device
            memset(rx_buf, 0, n_rx_buf_size);
            boost::asio::read(*psp, boost::asio::buffer(rx_buf, 1));
            if (rx_buf[0] != st_sync)
            {
                //printf ("Error sync byte\n");
                continue;
            }
            t_current_time = ros::Time::now();
        }
        catch (...)
        {
            //printf ("Error read sync byte\n");
            continue;
        }

        try
        {
            //std::cout << "rx_buf1: " << (int)(*(rx_buf + 1)) << std::endl;
            // read type byte of header from device
            boost::asio::read(*psp, boost::asio::buffer(rx_buf + 1, 1));
            switch (*(rx_buf + 1))
            {
            case 0x2:
                n_pkg_len = sizeof(Error_Frame);
                break;
            case 0x3:
                n_pkg_len = sizeof(Odom_Frame);
                break;
            case 0x4:
                n_pkg_len = sizeof(Sonar_Frame);
                break;
            case 0x5:
                n_pkg_len = sizeof(Volume_Frame);
                break;
            case 0x7:
                n_pkg_len = sizeof(Log_Frame);
                break;
            case 0x8:
                n_pkg_len = sizeof(Config_Frame);
                break;
            default:
                n_pkg_len = 0;
                //printf("Error command type:%d", n_pkg_len);
                continue;
            }

            // seg read left package data;
            boost::asio::read(*psp, boost::asio::buffer(rx_buf + 2, n_pkg_len - 2));
            
            // std::cout << "rx_buf[n_pkg_len - 1]: " << (int)(rx_buf[n_pkg_len - 1]) << std::endl;  //85
            // std::cout << "ed_sync: " << (int)ed_sync << std::endl;  //85
            // std::cout << "n_pkg_len: " << n_pkg_len << std::endl;   //38
            
            //  ???????????
            if (rx_buf[n_pkg_len - 1] != ed_sync)
            {
                // wrong stop byte package;
                printf("Error stop byte:%d \n", rx_buf[n_pkg_len - 1]);
                //printf ("Error stop byte\n");
                continue;
            }
            
            // check the check sum
            if (rx_buf[n_pkg_len - 2] != cal_check_sum(rx_buf, n_pkg_len))
            {

                // wrong check sum package;
                //printf ("Error check sum\n");
                continue;
            }

            //std::cout << "rx_buf2: " << (int)(*(rx_buf + 1)) << std::endl;
            bool res;
            
            // process package
            switch (*(rx_buf + 1))
            {
            case 0x2:
                res = error_frame_paser((Error_Frame *)rx_buf, t_current_time, config, ros_handlers, psp);
                break;
            case 0x3:
                res = odom_frame_paser((Odom_Frame *)rx_buf, t_current_time, config, ros_handlers);
                break;
            case 0x4:
                res = sonar_frame_paser((Sonar_Frame *)rx_buf, t_current_time, config, ros_handlers);
                break;
            case 0x5:
                res = volume_frame_paser((Volume_Frame *)rx_buf, t_current_time, config, ros_handlers);
                break;
            case 0x7:
                res = log_frame_paser((Log_Frame *)rx_buf, t_current_time, config, ros_handlers);
                break;
            case 0x8:
                res = config_ack_frame_paser((Config_Frame *)rx_buf, t_current_time, config,
                                             ros_handlers, config_mutex, n_command_index, f_command_value);
                break;
            }
        }
        catch (...)
        {
            continue;
        }
    }

    return;
}

void odom_callback(const geometry_msgs::Twist::ConstPtr &inMsg)
{
    // ros::Time recv = ros::Time::now();
    // std::lock_guard<std::mutex> lockG(lock_odom);
    inMsg_m = *inMsg;
    // m_last_recv_odom_time = recv;
    // m_odom_recv_initd = true;
}

void send_odom(const ros::Publisher &pb, const geometry_msgs::Twist &inMsg, double &recvTime)
{
    boost::system::error_code err;
    Control_Frame cFrame = {0};

    int n_sizeof_cFrame = sizeof(Control_Frame);

    // my_velocity_smoother(inMsg.linear.x, inMsg.angular.z, v_out, w_out);
    v_out = inMsg.linear.x;
    w_out = inMsg.angular.z;

    geometry_msgs::Twist msg;
    msg.linear.x = inMsg.linear.x;
    msg.linear.y = v_out;
    msg.angular.z = inMsg.angular.z;        // msg.angular.x = inMsg.angular.z ???
    msg.angular.y = w_out;
    pb.publish(msg);
    // cFrame.linear_vel = inMsg->linear.x;
    // cFrame.angular_vel = inMsg->angular.z;
    cFrame.linear_vel = v_out;
    cFrame.angular_vel = w_out;
    // printf  ("control status: %f\n", n_control_rates);
    cFrame.control_rate = n_control_rates;

    // printf  ("break status: %d\n", n_break_status);
    cFrame.control_bit = n_break_status == true ? 0x80000000 : 0x0;
    // printf  ("light status: %d\n", n_light_status);
    cFrame.control_bit |= (n_light_status & 0xFFFF); // only short type

    cFrame.header.sync = st_sync;
    cFrame.header.type = 0x1;
    cFrame.end.stop = ed_sync;

    cFrame.end.crc = cal_check_sum((unsigned char *)&cFrame, n_sizeof_cFrame);

    if (fabs(cFrame.linear_vel) > 0 || fabs(cFrame.angular_vel) > 0)
        ROS_INFOMA("send_odom,linear_vel:%f, angular_vel:%f, control_rate:0x %x, control_bit:0x %x,recvtime:%f",
                   cFrame.linear_vel, cFrame.angular_vel, cFrame.control_rate, cFrame.control_bit, recvTime);

#if 0
    for (int i = 0; i < n_sizeof_cFrame; i++) {
        printf ("%x ",  *((unsigned char *)&cFrame+ i));
    }


    printf ("\n");
#endif

    // try {
    //     if (psp == NULL) {
    //         printf ("Error of serial\n");
    //         return;
    //     }

    //     psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
    //     if (err) {
    //         psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
    //     }
    // } catch (...) {
    //     printf ("Error to write\n");
    // }
    try
    {
        if (psp_control == NULL)
        {
            ROS_INFOMA("Error of serial");
            return;
        }

        psp_control->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        if (err)
        {
            psp_control->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        }
    }
    catch (...)
    {
        ROS_INFOMA("Error to write");
    }
    return;
}

void set_break_status(const std_msgs::Bool::ConstPtr &inMsg, bool &n_break_status)
{
    n_break_status = inMsg->data;
    return;
}

void set_light_status(const std_msgs::Int16::ConstPtr &inMsg, boost::asio::serial_port *&psp, unsigned short &n_light_status)
{

    boost::system::error_code err;
    Light_Ctrl_Frame cFrame = {0};

    int n_sizeof_cFrame = sizeof(Light_Ctrl_Frame);

    if (n_light_status != inMsg->data)
    {
        cFrame.n_light_ctrl = inMsg->data;
    }
    else
    {
        return;
    }

    cFrame.header.sync = st_sync;
    cFrame.header.type = 0xE;
    cFrame.end.stop = ed_sync;

    cFrame.end.crc = cal_check_sum((unsigned char *)&cFrame, n_sizeof_cFrame);

    ROS_INFOMA("set_light_status()cFrame.n_light_ctrl:%d", cFrame.n_light_ctrl);
    try
    {
        if (psp == NULL)
        {
            ROS_INFOMA("Error of serial");
            return;
        }

        psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        if (err)
        {
            psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        }
    }
    catch (...)
    {
        ROS_INFOMA("Error to write");
    }
    return;
    return;
}

void set_control_rate(const std_msgs::Float32::ConstPtr &inMsg, float &n_control_rates)
{
    n_control_rates = inMsg->data;
    return;
}

bool set_stm32config_float(robot_msgs::set_stm32config_float::Request &req,
                           robot_msgs::set_stm32config_float::Response &res, boost::mutex &config_port_mutex, boost::timed_mutex &config_mutex, boost::asio::serial_port *psp,
                           int &n_command_index, float &f_command_value)
{

    Config_Frame config = {0};
    config.header.sync = st_sync;
    config.end.stop = ed_sync;
    config.header.type = 0x6;

    config.index = req.index;
    config.value = req.value;

    config.end.crc = cal_check_sum((unsigned char *)&config, sizeof(Config_Frame));

    boost::system::error_code err;
    config_port_mutex.lock();
    config_mutex.lock();
    ROS_INFOMA("set_stm32config_float():config.index:%d, config.value:%f",
               config.index, config.value);

    psp->write_some(boost::asio::buffer((unsigned char *)&config, sizeof(Config_Frame)), err);
    if (err)
    {
        psp->write_some(boost::asio::buffer((unsigned char *)&config, sizeof(Config_Frame)), err);
        if (err)
        {
            res.res = false;
            res.value = -1.0;
            config_mutex.unlock();
            config_port_mutex.unlock();
            return false;
        }
    }

    if (config_mutex.timed_lock(boost::posix_time::seconds(1)))
    {
        // ok
        if (n_command_index == config.index)
        {
            res.res = true;
            res.value = f_command_value;
        }
    }
    else
    {
        // can't get data;
        res.res = false;
        res.value = -1.0;
    }

    config_port_mutex.unlock();

    return true;
}

void robot_sonar_ctrl(const std_msgs::UInt8::ConstPtr &inMsg, boost::asio::serial_port *&psp, unsigned char &n_sonar_ctrl)
{
    boost::system::error_code err;
    Sonar_Ctrl_Frame cFrame = {0};

    int n_sizeof_cFrame = sizeof(Sonar_Ctrl_Frame);

    cFrame.n_sonar_ctrl = inMsg->data;

    cFrame.header.sync = st_sync;
    cFrame.header.type = 0xC;
    cFrame.end.stop = ed_sync;

    cFrame.end.crc = cal_check_sum((unsigned char *)&cFrame, n_sizeof_cFrame);

    try
    {
        if (psp == NULL)
        {
            ROS_INFOMA("Error of serial");
            return;
        }
        ROS_INFOMA("robot_sonar_ctrl():cFrame.n_sonar_ctrl:%d", cFrame.n_sonar_ctrl);
        psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        if (err)
        {
            psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        }
    }
    catch (...)
    {
        ROS_INFOMA("Error to write");
    }
    return;
}

void robot_charger_ctrl(const std_msgs::Bool::ConstPtr &inMsg, boost::asio::serial_port *&psp, bool &n_charger_ctrl)
{
    boost::system::error_code err;
    Charger_Ctrl_Frame cFrame = {0};

    int n_sizeof_cFrame = sizeof(Charger_Ctrl_Frame);

    // if (n_charger_ctrl != inMsg->data ) {
    cFrame.n_charger_ctrl = inMsg->data;
    ROS_INFOMA("true......................%d ", inMsg->data);
    //} else {
    //   return;
    //}

    cFrame.header.sync = st_sync;
    cFrame.header.type = 0xD;
    cFrame.end.stop = ed_sync;

    cFrame.end.crc = cal_check_sum((unsigned char *)&cFrame, n_sizeof_cFrame);

#if 1
    std::stringstream ss;
    for (int i = 0; i < n_sizeof_cFrame; i++)
    {
        ss << hex << *((unsigned char *)&cFrame + i) << " ";
    }

    ROS_INFOMA("robot_charger_ctrl():n_charger_ctrl:%d, &cFrame:  %s",
               cFrame.n_charger_ctrl, ss.str().c_str());
#endif

    try
    {
        if (psp == NULL)
        {
            ROS_INFOMA("Error of serial");
            return;
        }

        psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        if (err)
        {
            psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        }
    }
    catch (...)
    {
        ROS_INFOMA("Error to write");
    }
    return;
}

// lyy
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu tmp = *msg;
    geometry_msgs::Quaternion orientation = tmp.orientation;
    float imu_th_current = tf::getYaw(orientation);
    if (!imu_th_init)
    {
        imu_th_init = true;
        imu_th_last = imu_th_current;
    }
    float imu_th_delta = imu_th_current - imu_th_last;
    if (imu_th_delta > 6.0f)
    {
        imu_th_delta -= 6.2831853f;
    }
    else if (imu_th_delta < -6.0f)
    {
        imu_th_delta += 6.2831853f;
    }
    imu_th_last = imu_th_current;
    imu_th = odom_th_last + imu_th_delta;
}

int main(int argc, char *argv[])
{
    initSignalHandle();
    startLogger();
    ros::init(argc, argv, "amr_serial_port");
    ros::NodeHandle nh_("~");
    ros::NodeHandle nh;
    ROS_INFOMA("amr_serial_port start");
    Serial_Config config;
    std::string str_config_name;
    std::string str_control_name;

    nh_.param("tty_config_device", str_config_name, std::string("/dev/ttyUSB1"));   // /dev/amr_config
    nh_.param("tty_control_device", str_control_name, std::string("/dev/ttyUSB0"));   // /dev/amr_control

    nh_.param("odom_id", config.str_odom_id, std::string("odom"));
    nh_.param("robot_id", config.str_robot_id, std::string("base_footprint"));
    nh_.param("baud_rate", config.n_baud_rate, 115200);
    // nh_.param("baud_rate", config.n_baud_rate, 230400);
    nh_.param("n_char_size", config.n_char_size, 8);
    nh_.param("pub_tf", config.pub_tf, false);
    nh_.param("queue", config.n_queue_size, 5);
    nh_.param("rate", config.n_rate_speed, 10);
    nh_.param("sonar_fov", config.f_fov_sonar, 0.2);
    nh_.param("sonar_min_range", config.f_min_sonar, 0.25);
    nh_.param("sonar_max_range", config.f_max_sonar, 3.00);

    init_odom_msg(config);

    // boost::asio::serial_port *psp_control = NULL;
    // boost::asio::serial_port *psp_config  = NULL;

    Ros_Handlers ros_handlers;
    ros_handlers.pub_odom = nh.advertise<nav_msgs::Odometry>("/amr/robot_odom", config.n_queue_size);
    ros_handlers.pub_diag = nh.advertise<diagnostic_msgs::DiagnosticArray>("/amr/robot_diagnostic", config.n_queue_size);
    ros_handlers.pub_break = nh.advertise<robot_msgs::robot_break_status>("/amr/robot_break_status", config.n_queue_size);
    ros_handlers.pub_light = nh.advertise<robot_msgs::robot_light_status>("/amr/robot_light_status", config.n_queue_size);

    ros_handlers.pub_log = nh.advertise<robot_msgs::robot_debug_msg>("/amr/robot_debug_msg", config.n_queue_size);

    // add 10.15
    ros_handlers.pub_charger = nh.advertise<std_msgs::Bool>("/amr/robot_charger_status", config.n_queue_size);

    ros_handlers.pub_sonar_status = nh.advertise<std_msgs::UInt8>("/amr/robot_sonar_status", config.n_queue_size);

    ros_handlers.pub_sonar = nh.advertise<sensor_msgs::Range>("/amr/robot_sonar", config.n_queue_size);

    ros_handlers.sub_imu = nh.subscribe("/amr_imu", 10, &imu_callback); // lyy

    // bool n_break_status = true;
    // bool n_charger_ctrl = false;
    // unsigned char n_sonar_ctrl = 0xFF;
    // float n_control_rates = config.n_rate_speed;
    // unsigned short n_light_status = 0x0;

    // config config varialbes;
    int n_command_index = -1;
    float f_command_value = -1;
    boost::mutex config_port_mutex;
    boost::timed_mutex config_mutex;

    ros::Subscriber serial_sub = nh.subscribe<geometry_msgs::Twist>("/amr/cmd_vel_ctrl", config.n_queue_size, boost::bind(&odom_callback, _1)); // lyy
    ros::Subscriber break_sub = nh.subscribe<std_msgs::Bool>("/amr/robot_break_ctrl", config.n_queue_size, boost::bind(&set_break_status, _1, boost::ref(n_break_status)));
    ros::Subscriber light_sub = nh.subscribe<std_msgs::Int16>("/amr/robot_light_ctrl", config.n_queue_size, boost::bind(&set_light_status, _1, boost::ref(psp_control), boost::ref(n_light_status)));
    ros::Subscriber control_rate = nh.subscribe<std_msgs::Float32>("/amr/robot_rate_ctrl", config.n_queue_size, boost::bind(&set_control_rate, _1, boost::ref(n_control_rates)));

    ros::Subscriber sonar_ctrl = nh.subscribe<std_msgs::UInt8>("/arm/robot_sonar_ctrl", config.n_queue_size, boost::bind(&robot_sonar_ctrl, _1, boost::ref(psp_control), boost::ref(n_sonar_ctrl)));

    ros::Subscriber charger_ctyrl = nh.subscribe<std_msgs::Bool>("/amr/robot_charger_ctrl", config.n_queue_size, boost::bind(&robot_charger_ctrl, _1, boost::ref(psp_control), boost::ref(n_charger_ctrl)));

    ros::Publisher target_vel_pub = nh.advertise<geometry_msgs::Twist>("/target_vel_test", 10, true);

    ros::ServiceServer service = nh.advertiseService<robot_msgs::set_stm32config_float::Request, robot_msgs::set_stm32config_float::Response>("set_stm32config_float", boost::bind(&set_stm32config_float, _1, _2,
                                                                                                                                                                                   boost::ref(config_port_mutex), boost::ref(config_mutex), boost::ref(psp_config),
                                                                                                                                                                                   boost::ref(n_command_index), boost::ref(f_command_value)));

    boost::thread t_control(boost::bind(&serial_read_thread, boost::ref(config),
                                        boost::ref(str_control_name), boost::ref(ros_handlers), boost::ref(psp_control), boost::ref(config_mutex),
                                        boost::ref(n_command_index), boost::ref(f_command_value)));

    boost::thread t_config(boost::bind(&serial_read_thread, boost::ref(config),
                                       boost::ref(str_config_name), boost::ref(ros_handlers), boost::ref(psp_config), boost::ref(config_mutex),
                                       boost::ref(n_command_index), boost::ref(f_command_value)));
    // printf("finishing thread!!!!!\n");
    ros::Rate loop_rate(100); // lyy
    geometry_msgs::Twist temp_inMsg_m;
    double recv_time = 0;
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        {
            // std::lock_guard<std::mutex> lockG(lock_odom);
            // recv_time = m_last_recv_odom_time.toSec();
            // double time_diff = (ros::Time::now().toSec() - recv_time);
            // if(!m_odom_recv_initd)
            // {
            //     ROS_INFOMA("error()odom dont init");
            //     continue;
            // }
            // if( (fabs(inMsg_m.linear.x) > 0 || fabs(inMsg_m.angular.z) > 0) && (time_diff > 0.5))
            // {
            //     temp_inMsg_m.linear.x = temp_inMsg_m.angular.z = 0;
            //     ROS_INFOMA("error() long time (%f)not recv odom data,send vel 0 to stm32,last recv:%f,lastv,w:%f,%f",\
            //     time_diff,recv_time, inMsg_m.linear.x, inMsg_m.angular.z);
            // }
            // else
            temp_inMsg_m = inMsg_m;
        }

        send_odom(target_vel_pub, temp_inMsg_m, recv_time);

        // send_light(1);

        // ROS_INFOMA("loop is running!!!!!\n");
    }

    try
    {
        t_config.interrupt();
        t_control.interrupt();
        t_config.join();
        t_control.join();
    }
    catch (...)
    {
        return 1;
    }

    return 0;
}
