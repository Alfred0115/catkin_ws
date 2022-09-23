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

using namespace std;


unsigned char cal_check_sum(unsigned char *pdata, int n_size) {
    unsigned char tmp_sum = 0x0;

    for (int i = 0; i < n_size - 2; i++) {
        tmp_sum += *(unsigned char *)(pdata + i);
    }

    return tmp_sum;
}

bool error_frame_paser(Error_Frame *pError_Frame, ros::Time t_current_time,  Serial_Config &config, Ros_Handlers &ros_handlers, boost::asio::serial_port *psp) {
    diagnostic_msgs::DiagnosticArray error_arr;
    error_arr.header.stamp = t_current_time;
    error_arr.header.frame_id = "stm32";

    diagnostic_msgs::DiagnosticStatus  error_msg;
    error_msg.level = pError_Frame->value == 1 ? diagnostic_msgs::DiagnosticStatus::ERROR:diagnostic_msgs::DiagnosticStatus::OK; 
    error_msg.message = "hardware_id is the error number"; 
    error_msg.hardware_id = to_string(pError_Frame->value);
    error_arr.status.push_back(error_msg);
    ros_handlers.pub_diag.publish(error_arr);

    //write ack to serial port
    pError_Frame->header.type = 0x9;
    try {
        if (psp != NULL) {
            boost::system::error_code err;  
            psp->write_some(boost::asio::buffer((unsigned char *)pError_Frame, sizeof(Error_Frame)), err);
            if (err) {
                psp->write_some(boost::asio::buffer((unsigned char *)pError_Frame, sizeof(Error_Frame)), err);
            }
        }
    }catch (...) {
        return false;
    }
    
    return true;
}

nav_msgs::Odometry odom_msg;

void init_odom_msg(Serial_Config &config) {
    odom_msg.header.frame_id = config.str_odom_id;
    odom_msg.child_frame_id = config.str_robot_id;
    odom_msg.pose.pose.position.x = 0;
    odom_msg.pose.pose.position.y = 0;
    odom_msg.pose.pose.position.z = 0;

    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;
    odom_msg.pose.pose.orientation.w = 0;

    for (int i = 0; i < 36; i++ ) { 
        odom_msg.pose.covariance[i]  = 0.0;
    }   

    odom_msg.pose.covariance[0] = 0.1; // px
    odom_msg.pose.covariance[7] = 0.1; //py
    odom_msg.pose.covariance[14] = 0.0; //pz
    odom_msg.pose.covariance[21] = 0.0; //ax
    odom_msg.pose.covariance[28] = 0.0; //ay
    odom_msg.pose.covariance[35] = 0.1; //az

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    for (int i = 0; i < 36; i++ ) {
        odom_msg.twist.covariance[i] = 0.0;
    }

    odom_msg.twist.covariance[0] = 0.2; // vx
    odom_msg.twist.covariance[7] = 0.01; //vy
    odom_msg.twist.covariance[14] = 0.0; //vz
    odom_msg.twist.covariance[21] = 0.0; //wx
    odom_msg.twist.covariance[28] = 0.0; //wy
    odom_msg.twist.covariance[35] = 0.1; //wz
}

bool odom_frame_paser(Odom_Frame *pOdom_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers) {
    odom_msg.header.stamp = t_current_time;

    odom_msg.pose.pose.position.x = pOdom_Frame->position_x;
    odom_msg.pose.pose.position.y = pOdom_Frame->position_y;

    odom_msg.twist.twist.linear.x = pOdom_Frame->linear_vel;
    odom_msg.twist.twist.angular.z = pOdom_Frame->angular_vel;

    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR((float)pOdom_Frame->position_w, 0, 0); 

    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);

    odom_msg.pose.pose.orientation.x = q_tf.getX();;
    odom_msg.pose.pose.orientation.y = q_tf.getY();;
    odom_msg.pose.pose.orientation.z = q_tf.getZ();;
    odom_msg.pose.pose.orientation.w = q_tf.getW();;

    ros_handlers.pub_odom.publish(nav_msgs::Odometry(odom_msg));

    static tf::TransformBroadcaster br;
    tf::Transform transform;  
    transform.setOrigin( tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0) );
    transform.setRotation(q_tf);
    try {
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
    } catch (...) {
	    return false;
    }
    robot_msgs::robot_break_status robot_break;
    robot_break.header.stamp = t_current_time;
    robot_break.header.frame_id = "robot_break_status";


    robot_break.control_break = pOdom_Frame->robot_state & 0x1;
    robot_break.sonar_break  = pOdom_Frame->robot_state & 0x1<<1;
    robot_break.error_break  = pOdom_Frame->robot_state & 0x1<<2;
    robot_break.button_break  = pOdom_Frame->robot_state & 0x1<<3;
    ros_handlers.pub_break.publish(robot_break);

    robot_msgs::robot_light_status robot_light;
    robot_light.header.stamp = t_current_time;
    robot_light.header.frame_id = "robot_light_status";

    robot_light.light_pattern = pOdom_Frame->other_state &0x0000FFFF;
    ros_handlers.pub_light.publish(robot_light);
    return true;
}
bool sonar_frame_paser(Sonar_Frame *pSonar_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers) {
    sensor_msgs::Range sonar_data;
    sonar_data.header.stamp = t_current_time;
    sonar_data.field_of_view = config.f_fov_sonar;
    sonar_data.min_range = config.f_min_sonar;
    sonar_data.max_range = config.f_max_sonar;
    for (int i = 0; i < 6; i++) {
        sprintf((char *)sonar_data.header.frame_id.c_str(), "amr_sonar_id_0%d", i);
        sonar_data.range = *(&pSonar_Frame->range0 + i);
        ros_handlers.pub_sonar.publish(sensor_msgs::Range(sonar_data));
    }

    return true;
}

bool volume_frame_paser(Volume_Frame *pVolume_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers) {
    return true; 
}

bool log_frame_paser(Log_Frame *pLog_Frame, ros::Time t_current_time, Serial_Config &config, Ros_Handlers &ros_handlers) {
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
        Serial_Config &config, Ros_Handlers &ros_handlers, \
        boost::timed_mutex &config_mutex, int &n_command_index, float &f_command_value) {

    n_command_index = pConfig_ACK_Frame->index;
    f_command_value = pConfig_ACK_Frame->value;
    config_mutex.unlock();
    return true;
}

void serial_read_thread(Serial_Config &config, std::string &sz_dev_name, Ros_Handlers &ros_handlers, \
        boost::asio::serial_port *&psp, boost::timed_mutex &config_mutex, int &n_command_index, float &f_command_value) {
    const int n_rx_buf_size = 128;
    unsigned char rx_buf[n_rx_buf_size];

    ros::Rate rate(1);
    boost::asio::io_service io_s;
    ros::Time t_current_time;

    int n_pkg_len = 0;
    int n_pkg_header_len = sizeof(Frame_header);

    while (ros::ok()) {
        // seg connect the serial device
        try {
            if (psp == NULL) {
                printf ("Try to connect device : %s\n", sz_dev_name.c_str());
                psp = new boost::asio::serial_port(io_s, sz_dev_name);
                if (NULL == psp) {
                    rate.sleep();
                    continue;
                }

                psp->set_option(boost::asio::serial_port::baud_rate(config.n_baud_rate)); 
                psp->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
                psp->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                psp->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
                psp->set_option(boost::asio::serial_port::character_size(config.n_char_size));

            }
        } catch (...) {
            if (psp != NULL) {
                delete psp;
                psp == NULL;
            }
            rate.sleep();
            continue;
        }

        try{
            // seg read sync byte of header from device
            memset(rx_buf, 0, n_rx_buf_size);
            boost::asio::read(*psp, boost::asio::buffer(rx_buf, 1));
            if (rx_buf[0] != st_sync) {
                //printf ("Error sync byte\n");
                continue;
            }
            t_current_time = ros::Time::now(); 

        } catch (...) {
            //printf ("Error read sync byte\n");
            continue;
        }

        try {
            // read type byte of header from device
            boost::asio::read(*psp, boost::asio::buffer(rx_buf+1, 1));
            switch (*(rx_buf+1)) {
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
                    //printf ("Error command type\n");
                    continue;
            }

            // seg read left package data;
            boost::asio::read(*psp, boost::asio::buffer(rx_buf+2, n_pkg_len - 2));
            if (rx_buf[n_pkg_len-1] != ed_sync) {
                //wrong stop byte package;
                //printf ("Error stop byte\n");
                continue;
            }

            // check the check sum
            if (rx_buf[n_pkg_len-2] != cal_check_sum (rx_buf, n_pkg_len)) {

                //wrong check sum package;
                //printf ("Error check sum\n");
                continue;
            }

            bool res;
            //printf("get New packge");
            // process package
            switch (*(rx_buf+1)) {
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
                    res = volume_frame_paser ((Volume_Frame *)rx_buf, t_current_time, config, ros_handlers);
                    break;
                case 0x7:
                    res = log_frame_paser((Log_Frame *)rx_buf, t_current_time, config, ros_handlers);
                    break;
                case 0x8:
                    res = config_ack_frame_paser((Config_Frame*)rx_buf, t_current_time, config, \
                            ros_handlers, config_mutex, n_command_index, f_command_value);
                    break;
            }            
        } catch (...) {
            continue;
        }
    }
    
    return;
}

void send_odom(const geometry_msgs::Twist::ConstPtr& inMsg, boost::asio::serial_port *&psp, bool &n_break_status, unsigned short &n_light_status, float &n_control_rates) {

    boost::system::error_code err;  
    Control_Frame cFrame = {0};

    int n_sizeof_cFrame = sizeof(Control_Frame);

    cFrame.linear_vel = inMsg->linear.x;
    cFrame.angular_vel = inMsg->angular.z;
    //printf  ("control status: %f\n", n_control_rates);
    cFrame.control_rate = n_control_rates;

    //printf  ("break status: %d\n", n_break_status);
    cFrame.control_bit = n_break_status == true ? 0x80000000:0x0;
    //printf  ("light status: %d\n", n_light_status);
    cFrame.control_bit |= (n_light_status&0xFFFF); // only short type

    cFrame.header.sync = st_sync;
    cFrame.header.type = 0x1;
    cFrame.end.stop = ed_sync;



    cFrame.end.crc = cal_check_sum((unsigned char *)&cFrame, n_sizeof_cFrame);

#if 0
    for (int i = 0; i < n_sizeof_cFrame; i++) {
        printf ("%x ",  *((unsigned char *)&cFrame+ i));
    }


    printf ("\n");
#endif

    try {
        if (psp == NULL) {
            printf ("Error of serial\n");
            return;
        }

        psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        if (err) {
            psp->write_some(boost::asio::buffer((unsigned char *)&cFrame, n_sizeof_cFrame), err);
        }
    } catch (...) {
        printf ("Error to write\n");
    }
    return;
}

void set_break_status(const std_msgs::Bool::ConstPtr& inMsg, bool &n_break_status) {
    n_break_status = inMsg->data;
    return;
}

void set_light_status(const std_msgs::Int16::ConstPtr& inMsg, unsigned short &n_light_status) {
    n_light_status = inMsg->data;
    return;
}

void set_control_rate(const std_msgs::Float32::ConstPtr& inMsg, float &n_control_rates) {
    n_control_rates = inMsg->data;
    return;
}

bool set_stm32config_float(robot_msgs::set_stm32config_float::Request &req, \
        robot_msgs::set_stm32config_float::Response &res,boost::mutex \
        &config_port_mutex, boost::timed_mutex &config_mutex, boost::asio::serial_port *psp, \
        int &n_command_index, float &f_command_value ) {
    
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
    psp->write_some(boost::asio::buffer((unsigned char *)&config, sizeof(Config_Frame)), err);
    if (err) {
        psp->write_some(boost::asio::buffer((unsigned char *)&config, sizeof(Config_Frame)), err);
        if (err) {
            res.res = false;
            res.value = -1.0;
            config_mutex.unlock();
            config_port_mutex.unlock();
            return false;
        }
    }

    if (config_mutex.timed_lock(boost::posix_time::seconds(1))) {
        //ok
        if (n_command_index == config.index ) {
            res.res = true;
            res.value = f_command_value;
        }
    } else {
        //can't get data;
        res.res = false;
        res.value = -1.0;
    }

    config_port_mutex.unlock();

    return true;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "amr_serial_port");

    ros::NodeHandle nh_("~");
    ros::NodeHandle nh;

    Serial_Config config;
    std::string str_config_name;
    std::string str_control_name;

    nh_.param("tty_config_device",  str_config_name,   std::string("/dev/amr_config"));
    nh_.param("tty_control_device", str_control_name, std::string("/dev/amr_control"));

    nh_.param("odom_id", config.str_odom_id, std::string("odom"));
    nh_.param("robot_id", config.str_robot_id, std::string("base_footprint"));
    nh_.param("baud_rate", config.n_baud_rate, 115200);
    //nh_.param("baud_rate", config.n_baud_rate, 230400);
    nh_.param("n_char_size", config.n_char_size, 8);
    nh_.param("pub_tf", config.pub_tf, false);
    nh_.param("queue", config.n_queue_size, 5);
    nh_.param("rate", config.n_rate_speed, 10);
    nh_.param("sonar_fov", config.f_fov_sonar, 0.2);
    nh_.param("sonar_min_range", config.f_min_sonar, 0.25);
    nh_.param("sonar_max_range", config.f_max_sonar, 3.00);

    init_odom_msg(config);

    boost::asio::serial_port *psp_control = NULL;
    boost::asio::serial_port *psp_config  = NULL;

    Ros_Handlers ros_handlers;
    ros_handlers.pub_odom = nh.advertise<nav_msgs::Odometry>\
                            ("/amr/robot_odom", config.n_queue_size);
    ros_handlers.pub_diag = nh.advertise<diagnostic_msgs::DiagnosticArray>\
                            ("/amr/robot_diagnostic", config.n_queue_size);
    ros_handlers.pub_break = nh.advertise<robot_msgs::robot_break_status> \
                             ("/amr/robot_break_status",config.n_queue_size); 
    ros_handlers.pub_light = nh.advertise<robot_msgs::robot_light_status> \
                             ("/amr/robot_light_status",config.n_queue_size); 

    ros_handlers.pub_log = nh.advertise<robot_msgs::robot_debug_msg> \
                           ("/amr/robot_debug_msg", config.n_queue_size);
    bool n_break_status = true;
    float n_control_rates = config.n_rate_speed;
    unsigned short n_light_status = 0x0;

    //config config varialbes;
    int n_command_index = -1;
    float f_command_value = -1;
    boost::mutex config_port_mutex;
    boost::timed_mutex config_mutex;

    ros::Subscriber serial_sub = nh.subscribe<geometry_msgs::Twist>\
                                 ("/amr/cmd_vel_ctrl", config.n_queue_size, boost::bind(&send_odom,  _1,  \
                                 boost::ref(psp_control), boost::ref(n_break_status), boost::ref(n_light_status), boost::ref(n_control_rates)));
    ros::Subscriber break_sub = nh.subscribe<std_msgs::Bool>\
                                ("/amr/robot_break_ctrl", config.n_queue_size, boost::bind(&set_break_status,  _1,  boost::ref(n_break_status)));
    ros::Subscriber light_sub = nh.subscribe<std_msgs::Int16>\
                                ("/amr/robot_light_ctrl", config.n_queue_size, boost::bind(&set_light_status,  _1,  boost::ref(n_light_status)));
    ros::Subscriber control_rate = nh.subscribe<std_msgs::Float32>\
                                ("/amr/robot_rate_ctrl", config.n_queue_size, boost::bind(&set_control_rate,  _1,  boost::ref(n_control_rates)));

    ros::ServiceServer service = nh.advertiseService \
                                 <robot_msgs::set_stm32config_float::Request, robot_msgs::set_stm32config_float::Response>\
                                 ("set_stm32config_float", boost::bind(&set_stm32config_float, _1, _2, \
                                 boost::ref(config_port_mutex), boost::ref(config_mutex), boost::ref(psp_config), \
                                 boost::ref(n_command_index), boost::ref(f_command_value)));

    boost::thread t_control(boost::bind(&serial_read_thread, boost::ref(config), \
                boost::ref(str_control_name), boost::ref(ros_handlers), boost::ref(psp_control),boost::ref(config_mutex), \
                boost::ref(n_command_index), boost::ref(f_command_value)));

    boost::thread t_config(boost::bind(&serial_read_thread, boost::ref(config), \
                boost::ref(str_config_name), boost::ref(ros_handlers), boost::ref(psp_config), boost::ref(config_mutex), \
                boost::ref(n_command_index), boost::ref(f_command_value)));

    while (ros::ok()) {
        ros::spin();
    }

    try {
        t_config.interrupt();
        t_control.interrupt();
        t_config.join();
        t_control.join();
    } catch (...) {
        return 1;
    }

    return 0;
}
