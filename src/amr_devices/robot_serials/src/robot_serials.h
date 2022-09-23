#ifndef __ROBOT_SERIAL_POSR__
#define __ROBOT_SERIAL_POSR__

#include <ros/ros.h>

unsigned char st_sync = 0xAB;
unsigned char ed_sync = 0x55;

typedef struct {
    std::string str_odom_id;
    std::string str_robot_id;

    int n_baud_rate;
    int n_char_size;
    int n_rate_speed;
    int n_queue_size;
    double f_fov_sonar;
    double f_min_sonar;
    double f_max_sonar;

    bool pub_tf;
}Serial_Config;

typedef struct {
    ros::Publisher  pub_odom;
    ros::Publisher  pub_diag;
    ros::Publisher  pub_log;
    ros::Publisher  pub_light;
    ros::Publisher  pub_break;
    ros::Publisher  pub_sonar;
    ros::Publisher  pub_charger;
    ros::Publisher  pub_sonar_status;
    ros::Subscriber  sub_imu;//lyy
}Ros_Handlers;

typedef struct{
    unsigned char sync;
    unsigned char type;

}__attribute__ ((packed)) Frame_header;

typedef struct {
    unsigned char crc;
    unsigned char stop;
}__attribute__ ((packed)) Frame_end;

//0x1
typedef struct {
    Frame_header header;
    float linear_vel;
	float angular_vel;
	float control_rate;
	unsigned int control_bit;
    Frame_end end;
}__attribute__ ((packed)) Control_Frame;

//0x2 0x9
typedef struct {
    Frame_header header;
	unsigned int index;
	unsigned char value;
    Frame_end end;
}__attribute__ ((packed)) Error_Frame;

//0x3
typedef struct {
    Frame_header header __attribute__ ((aligned (2)));
	float position_x;
	float position_y;
	float position_w;
	
	float linear_vel;
	float angular_vel;
	unsigned int robot_state;
	unsigned int other_state;

    unsigned short err_code_l;
    unsigned short err_code_r;
    unsigned short urlter_wave_status;

    float range1;   // ??
	float range2;
	float range3;
	float range4;
	float range5;
    float range6;
    float range7;
    float range8;
    Frame_end end __attribute__ ((aligned (2)));
}__attribute__ ((packed)) Odom_Frame;


//0x4
typedef struct {
    Frame_header header;
	unsigned char index;
	float range0;
	float range1;
	float range2;
	float range3;
	float range4;
	float range5;
    Frame_end end;
}__attribute__ ((packed)) Sonar_Frame;

//0x5
typedef struct {
    Frame_header header;
	float voltage;
	float mAh;
	float tempure;
    Frame_end end;
}__attribute__ ((packed)) Volume_Frame;

//0x6 0x8
typedef struct {
    Frame_header header;
	unsigned short index;
	float value;
    Frame_end end;
}__attribute__ ((packed)) Config_Frame;

//0x7
typedef struct {
    Frame_header header;
	float L_wv;
	float R_wv;
	float linear_vel;
	float angular_vel;
	float linear_acc;
	float angular_acc;
	float position_x;
	float position_y;
	float position_w;
    Frame_end end;
}__attribute__ ((packed)) Log_Frame;


//0xC
typedef struct {
    Frame_header header;
    unsigned char n_sonar_ctrl;
    Frame_end end;
}__attribute__ ((packed)) Sonar_Ctrl_Frame;

//0xD
typedef struct {
    Frame_header header;
    int n_charger_ctrl;
    Frame_end end;
}__attribute__ ((packed)) Charger_Ctrl_Frame;

//0xE
typedef struct {
    Frame_header header;
    unsigned short n_light_ctrl;
    Frame_end end;
}__attribute__ ((packed)) Light_Ctrl_Frame;


#endif //__ROBOT_SERIAL_POSR__
