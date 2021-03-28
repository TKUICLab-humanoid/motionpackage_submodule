#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "ros/ros.h"
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include <vector>
#include <boost/program_options.hpp>
#include "../../cssl/cssl.h"
#include "../../cssl/port.h"
#include "../../cssl/cssl.c"
#include "tku_libs/TKU_tool.h"

#include "tku_msgs/Parameter_message.h"
#include "tku_msgs/Walking_message.h"
#include "tku_msgs/HeadPackage.h"
#include "tku_msgs/SandHandSpeed.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tku_msgs/InterfaceSend2Sector.h"
#include "tku_msgs/SingleMotorData.h"
#include "tku_msgs/SaveMotionVector.h"
#include "tku_msgs/SaveMotion.h"
#include "tku_msgs/callback.h"
#include "tku_msgs/CheckSector.h"
#include "tku_msgs/ReadMotion.h"
#include "tku_msgs/SensorPackage.h"
#include "tku_msgs/SensorSet.h"

//---save
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;
//---save

typedef struct
{
    int ID;
    int GoalPosition;
    int exGoalPosition;
    int Speed;

}tsRobotis;

#define AX12Velocity 1016949.152542373
#define offset1 6
#define offset2 21
#define offset3 46
#define offset4 61
// #define SENSOR_BUF_SIZE 60
#define SENSOR_SET_PACKAGE_SIZE 12
#define IMU_PACKAGE_SIZE 28

using std::string;
cssl_t *serial;
cssl_t *serial_head;
cssl_t *serial_IMU;
ros::Publisher FPGAack_Publish;
ros::Publisher InterfaceCallBack_Publish;
ros::Publisher walkack_Publish;
ros::Publisher ExecuteCallBack_Publish;
ros::Publisher Sensorpackage_Publish;

std_msgs::Bool walkack;
std_msgs::Bool interface_ack;
std_msgs::Bool execute_ack;
std_msgs::Int16 FPGAack;
vector<unsigned int> SaveSectorPackage;
vector<unsigned int> SendSectorPackage;
vector<unsigned int> handspeedpackage;
vector<unsigned char> IMUPackage;
vector<int> CheckSectorPackage;
tku_msgs::SaveMotionVector MotionSaveData;
//std::string SendSectorPackage;
vector<tsRobotis> RobotisList;
TimeClass timer_sensor_set(10);
struct timeval tstart, tend;
double timeuse;
uint8_t parameterpackage[31] = {0};
uint8_t motorpackage[19] = {0};
uint8_t HeadPackage[32] = {0};
uint8_t sectorpackage[5] = {0};
uint8_t torquePackage[13] = {0};
uint8_t onemotorpackage[87] = {0};
uint8_t packageMotorData[87] = {0}; 	// address||R MotorSum (id angleL angleH speedL speedH)*MotorSum DelayL DelayH checksum2
uint8_t packageEnd[2] = {0x4E, 0x45};	// N E
uint8_t sensorsetpackage[SENSOR_SET_PACKAGE_SIZE] = {0};
unsigned int savemotor9[3] = {0};
int InterfaceFlag = 0;


bool isBufFull = false;
int sensor_data_buf_cnt = 0;
uint8_t sensor_data_buf[IMU_PACKAGE_SIZE] = {0};

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
void Sensor_Data_Process();

bool fall_Down_Flag = false;
bool old_fall_Down_Flag = false;

int Desire_Roll = 0;
int Desire_Pitch = 0;
int Desire_Yaw = 0;
/*int Gain_Roll = 0;
int Gain_Pitch = 0;
int Gain_KP = 0;
int Gain_KD = 0;*/

int read_IMU_count = 0;

ToolInstance *tool = ToolInstance::getInstance();
