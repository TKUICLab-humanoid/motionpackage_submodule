#include "motionpackage/motionpackage.h"
#include <ros/callback_queue.h>

static void head_callback(int id, uint8_t *buf, int length)
{

}

static void IMU_callback(int id, uint8_t *buf, int length)
{
    bool got_imu_package_flag = false;
    int index = 0;

    for(int i=0; i<length; i++)
        IMUPackage.push_back(buf[i]);
    
    if(IMUPackage.size() >= IMU_PACKAGE_SIZE)
    {
        for(int i=0; i<IMUPackage.size(); i++)
        {
            if(IMUPackage[i] == 0x53 && IMUPackage[i+1] == 0x54 && IMUPackage[i+2] == 0xF7 && IMUPackage[i+IMU_PACKAGE_SIZE-1])
            index = i;
            got_imu_package_flag = true;
        }

        if(got_imu_package_flag)
        {
            for(int i=0; i<IMU_PACKAGE_SIZE; i++)
            {
                sensor_data_buf[i] = IMUPackage[index+i];
            }
        }
        Sensor_Data_Process();
        IMUPackage.clear();
    }
}

static void mcssl_callback(int id, uint8_t *buf, int length)
{
    static int dio_tmpstatus = 0;
    static bool walkdata_receive = false;
    printf("FPGAack is :%x %x %x %x \n",buf[0],buf[1],buf[2],buf[3]);
    for(int i = 0; i < 4; i++)
    {
        switch (dio_tmpstatus) {
        case 0:
            if(buf[i] == 'S')
                dio_tmpstatus = 1;
            break;
        case 1:
            if(buf[i] == 'U')
                dio_tmpstatus = 2;

            if(buf[i] == 'T')
                dio_tmpstatus = 4;
            break;
        case 2:
            dio_tmpstatus = 3;
            FPGAack.data = buf[i];
            FPGAack_Publish.publish(FPGAack);
            break;
        case 3:
            if(buf[i] == 'E')
            {
                dio_tmpstatus = 0;
                if(walkdata_receive)
                {
                    walkack.data = true;
                    walkack_Publish.publish(walkack);
                }
            }
            break;
        case 4:
            if(buf[i] == 'Y')
            {
                InterfaceFlag = 1;
            }
            else if(buf[i] == 'N')
            {
                InterfaceFlag = 0;
                SendSectorPackage.clear();
            }
            else if(buf[i] == 0xF5)
            {
                walkdata_receive = true;
            }
            else
            {
                walkdata_receive = false;
            }
            dio_tmpstatus = 3;
            break;
        }
    }
}

int mcssl_init()
{
    char *devs;
    char *devs_head;
    char *devs_IMU;
    cssl_start();
    if(!strcmp(tool->standPath, "/home/iclab/Desktop/Standmotion"))
    {
        devs="/dev/ttyUSB0";
        devs_head="/dev/ttyS1";
        devs_IMU="/dev/ttyS0";
    }
    else
    {
        devs="/dev/ttyUSB1";
        devs_head="/dev/ttyUSB2";
        devs_IMU="/dev/ttyUSB0";
    }
    if (!serial)
    {
        serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial_head)
    {
        serial_head=cssl_open(devs_head, head_callback, 0, 115200, 8, 0, 1);
    }
    if (!serial_IMU)
    {
        serial_IMU=cssl_open(devs_IMU, IMU_callback, 0, 115200, 8, 0, 1);
    }
    printf("Initialize Motion with port=%s...\n",devs);
    printf("Initialize Head with port=%s...\n",devs_head);
    printf("Initialize IMU with port=%s...\n",devs_IMU);
    if (!serial)
    {
        printf("%s\n",cssl_geterrormsg());
        printf("---> Motion RS232 OPEN FAIL <---\n");
        fflush(stdout);
        return -1;
    }
    if (!serial_head)
    {
        printf("%s\n",cssl_geterrormsg());
        printf("---> Head RS232 OPEN FAIL <---\n");
        fflush(stdout);
        return -1;
    }
    if (!serial_IMU)
    {
        printf("%s\n",cssl_geterrormsg());
        printf("---> Head RS232 OPEN FAIL <---\n");
        fflush(stdout);
        return -1;
    }
    cssl_setflowcontrol(serial, 0, 0);
    cssl_setflowcontrol(serial_head, 0, 0);
    cssl_setflowcontrol(serial_IMU, 0, 0);
    return 1;
}

void mcssl_finish()
{

    cssl_close(serial);
    cssl_close(serial_head);
    cssl_close(serial_IMU);
    cssl_stop();
}
//------------------------

void packageinit()
{
    parameterpackage[0] = 0x53;
    parameterpackage[1] = 0x54;
    parameterpackage[2] = 0xF5;
    parameterpackage[5] = 6;
    parameterpackage[30] = 0x45;

    motorpackage[0] = 0x53;
    motorpackage[1] = 0x54;
    motorpackage[2] = 0xF5;
    motorpackage[3] = 1;
    motorpackage[5] = 3;    // data length is 3 bytes
    motorpackage[18] = 0x45;
}

void paradata_send2fpga(int walking_mode, double x_swing, double y_swing, double z_swing, int period_t1, int period_t2, int sample_time, double lock_range, double base_default_z, double y_swing_shift, double x_swing_com, double base_lift_z)
{
    uint8_t x_swing_h, x_swing_l, y_swing_h, y_swing_l, z_swing_h, z_swing_l;
    uint8_t period_t1_h, period_t1_l, period_t2_h, period_t2_l;
    uint8_t base_default_z_h, base_default_z_l;
    uint8_t y_swing_shift_h, y_swing_shift_l;
    uint8_t x_swing_com_h, x_swing_com_l;
    uint8_t base_lift_z_h, base_lift_z_l;

    if(x_swing < 0)
    {
        x_swing_h = ((((int)(x_swing * -100.0)) >> 8) & 0xFF) | 0x80;
        x_swing_l = ((int)(x_swing * -100.0)) & 0xFF;
    }
    else
    {
        x_swing_h = (((int)(x_swing * 100.0)) >> 8) & 0xFF;
        x_swing_l = ((int)(x_swing * 100.0)) & 0xFF;
    }

    if(y_swing < 0)
    {
        y_swing_h = ((((int)(y_swing * -100.0)) >> 8) & 0xFF) | 0x80;
        y_swing_l = ((int)(y_swing * -100.0)) & 0xFF;
    }
    else
    {
        y_swing_h = (((int)(y_swing * 100.0)) >> 8) & 0xFF;
        y_swing_l = ((int)(y_swing * 100.0)) & 0xFF;
    }

    if(z_swing < 0)
    {
        z_swing_h = ((((int)(z_swing * -100.0)) >> 8) & 0xFF) | 0x80;
        z_swing_l = ((int)(z_swing * -100.0)) & 0xFF;
    }
    else
    {
        z_swing_h = (((int)(z_swing * 100.0)) >> 8) & 0xFF;
        z_swing_l = ((int)(z_swing * 100.0)) & 0xFF;
    }

    if(x_swing_com < 0)
    {
        x_swing_com_h = ((((int)(x_swing_com * -100.0)) >> 8) & 0xFF) | 0x80;
        x_swing_com_l = ((int)(x_swing_com * -100.0)) & 0xFF;
    }
    else
    {
        x_swing_com_h = (((int)(x_swing_com * 100.0)) >> 8) & 0xFF;
        x_swing_com_l = ((int)(x_swing_com * 100.0)) & 0xFF;
    }

    if(y_swing_shift < 0)
    {
        y_swing_shift_h = ((((int)(y_swing_shift * -100.0)) >> 8) & 0xFF) | 0x80;
        y_swing_shift_l = ((int)(y_swing_shift * -100.0)) & 0xFF;
    }
    else
    {
        y_swing_shift_h = (((int)(y_swing_shift * 100.0)) >> 8) & 0xFF;
        y_swing_shift_l = ((int)(y_swing_shift * 100.0)) & 0xFF;
    }

    if(base_lift_z < 0)
    {
        base_lift_z_h = ((((int)(base_lift_z * -100.0)) >> 8) & 0xFF) | 0x80;
        base_lift_z_l = ((int)(base_lift_z * -100.0)) & 0xFF;
    }
    else
    {
        base_lift_z_h = (((int)(base_lift_z * 100.0)) >> 8) & 0xFF;
        base_lift_z_l = ((int)(base_lift_z * 100.0)) & 0xFF;
    }

    base_default_z_h = (((int)(base_default_z * 100.0)) >> 8) & 0xFF;
    base_default_z_l = ((int)(base_default_z * 100.0)) & 0xFF;

    period_t1_h = (period_t1 >> 8) & 0xFF;
    period_t1_l = period_t1 & 0xFF;

    period_t2_h = (period_t2 >> 8) & 0xFF;
    period_t2_l = period_t2 & 0xFF;

    parameterpackage[6] = x_swing_h;
    parameterpackage[7] = x_swing_l;
    parameterpackage[8] = y_swing_h;
    parameterpackage[9] = y_swing_l;
    parameterpackage[10] = z_swing_h;
    parameterpackage[11] = z_swing_l;
    parameterpackage[12] = period_t1_h;
    parameterpackage[13] = period_t1_l;
    parameterpackage[14] = period_t2_h;
    parameterpackage[15] = period_t2_l;
    parameterpackage[16] = sample_time & 0xFF;
    parameterpackage[17] = ((int)(lock_range * 100.0)) & 0xFF;
    parameterpackage[18] = base_default_z_h;
    parameterpackage[19] = base_default_z_l;
    parameterpackage[20] = x_swing_com_h;
    parameterpackage[21] = x_swing_com_l;
    parameterpackage[22] = y_swing_shift_h;
    parameterpackage[23] = y_swing_shift_l;
    parameterpackage[24] = base_lift_z_h;
    parameterpackage[25] = base_lift_z_l;
    parameterpackage[26] = walking_mode & 0xFF;
    parameterpackage[27] = 0;   //reserve
    parameterpackage[28] = 0;   //reserve
    parameterpackage[29] = 0;   //reserve

    cssl_putdata(serial, parameterpackage, 31);
}

void paradata_send2fpga(int walking_mode, double y_swing, int period_t1, double kick_point_x, double kick_point_y, double kick_point_z, double back_point_x, double back_point_z, double support_foot_upper_hip_pitch, double kick_foot_ankle_upper_pitch, double support_foot_ankle_upper_pitch)
{
    uint8_t y_swing_h, y_swing_l;
    uint8_t period_t1_h, period_t1_l;
    uint8_t kick_point_x_h, kick_point_x_l;
    uint8_t kick_point_y_h, kick_point_y_l;
    uint8_t kick_point_z_h, kick_point_z_l;
    uint8_t back_point_x_h, back_point_x_l;
    uint8_t back_point_z_h, back_point_z_l;
    uint8_t support_foot_hip_upper_pitch_h, support_foot_hip_upper_pitch_l;
    uint8_t kick_foot_ankle_upper_pitch_h, kick_foot_ankle_upper_pitch_l;
    uint8_t support_foot_ankle_upper_pitch_h, support_foot_ankle_upper_pitch_l;

    if(y_swing < 0)
    {
        y_swing_h = ((((int)(y_swing * -100.0)) >> 8) & 0xFF) | 0x80;
        y_swing_l = ((int)(y_swing * -100.0)) & 0xFF;
    }
    else
    {
        y_swing_h = (((int)(y_swing * 100.0)) >> 8) & 0xFF;
        y_swing_l = ((int)(y_swing * 100.0)) & 0xFF;
    }
    if(kick_point_x < 0)
    {
        kick_point_x_h = ((((int)(kick_point_x * -100.0)) >> 8) & 0xFF) | 0x80;
        kick_point_x_l = ((int)(kick_point_x * -100.0)) & 0xFF;
    }
    else
    {
        kick_point_x_h = (((int)(kick_point_x * 100.0)) >> 8) & 0xFF;
        kick_point_x_l = ((int)(kick_point_x * 100.0)) & 0xFF;
    }
    if(kick_point_y < 0)
    {
        kick_point_y_h = ((((int)(kick_point_y * -100.0)) >> 8) & 0xFF) | 0x80;
        kick_point_y_l = ((int)(kick_point_y * -100.0)) & 0xFF;
    }
    else
    {
        kick_point_y_h = (((int)(kick_point_y * 100.0)) >> 8) & 0xFF;
        kick_point_y_l = ((int)(kick_point_y * 100.0)) & 0xFF;
    }
    if(kick_point_z < 0)
    {
        kick_point_z_h = ((((int)(kick_point_z * -100.0)) >> 8) & 0xFF) | 0x80;
        kick_point_z_l = ((int)(kick_point_z * -100.0)) & 0xFF;
    }
    else
    {
        kick_point_z_h = (((int)(kick_point_z * 100.0)) >> 8) & 0xFF;
        kick_point_z_l = ((int)(kick_point_z * 100.0)) & 0xFF;
    }
    if(back_point_x < 0)
    {
        back_point_x_h = ((((int)(back_point_x * -100.0)) >> 8) & 0xFF) | 0x80;
        back_point_x_l = ((int)(back_point_x * -100.0)) & 0xFF;
    }
    else
    {
        back_point_x_h = (((int)(back_point_x * 100.0)) >> 8) & 0xFF;
        back_point_x_l = ((int)(back_point_x * 100.0)) & 0xFF;
    }
    if(back_point_z < 0)
    {
        back_point_z_h = ((((int)(back_point_z * -100.0)) >> 8) & 0xFF) | 0x80;
        back_point_z_l = ((int)(back_point_z * -100.0)) & 0xFF;
    }
    else
    {
        back_point_z_h = (((int)(back_point_z * 100.0)) >> 8) & 0xFF;
        back_point_z_l = ((int)(back_point_z * 100.0)) & 0xFF;
    }
    if(support_foot_upper_hip_pitch < 0)
    {
        support_foot_hip_upper_pitch_h = ((((int)(support_foot_upper_hip_pitch * -100.0)) >> 8) & 0xFF) | 0x80;
        support_foot_hip_upper_pitch_l = ((int)(support_foot_upper_hip_pitch * -100.0)) & 0xFF;
    }
    else
    {
        support_foot_hip_upper_pitch_h = (((int)(support_foot_upper_hip_pitch * 100.0)) >> 8) & 0xFF;
        support_foot_hip_upper_pitch_l = ((int)(support_foot_upper_hip_pitch * 100.0)) & 0xFF;
    }
    if(kick_foot_ankle_upper_pitch < 0)
    {
        kick_foot_ankle_upper_pitch_h = ((((int)(kick_foot_ankle_upper_pitch * -100.0)) >> 8) & 0xFF) | 0x80;
        kick_foot_ankle_upper_pitch_l = ((int)(kick_foot_ankle_upper_pitch * -100.0)) & 0xFF;
    }
    else
    {
        kick_foot_ankle_upper_pitch_h = (((int)(kick_foot_ankle_upper_pitch * 100.0)) >> 8) & 0xFF;
        kick_foot_ankle_upper_pitch_l = ((int)(kick_foot_ankle_upper_pitch * 100.0)) & 0xFF;
    }

    if(support_foot_ankle_upper_pitch < 0)
    {
        support_foot_ankle_upper_pitch_h = ((((int)(support_foot_ankle_upper_pitch * -100.0)) >> 8) & 0xFF) | 0x80;
        support_foot_ankle_upper_pitch_l = ((int)(support_foot_ankle_upper_pitch * -100.0)) & 0xFF;
    }
    else
    {
        support_foot_ankle_upper_pitch_h = (((int)(support_foot_ankle_upper_pitch * 100.0)) >> 8) & 0xFF;
        support_foot_ankle_upper_pitch_l = ((int)(support_foot_ankle_upper_pitch * 100.0)) & 0xFF;
    }

    period_t1_h = (period_t1 >> 8) & 0xFF;
    period_t1_l = period_t1 & 0xFF;

    parameterpackage[6] = y_swing_h;
    parameterpackage[7] = y_swing_l;
    parameterpackage[8] = period_t1_h;
    parameterpackage[9] = period_t1_l;
    parameterpackage[10] = kick_point_x_h;
    parameterpackage[11] = kick_point_x_l;
    parameterpackage[12] = kick_point_y_h;
    parameterpackage[13] = kick_point_y_l;
    parameterpackage[14] = kick_point_z_h;
    parameterpackage[15] = kick_point_z_l;
    parameterpackage[16] = back_point_x_h;
    parameterpackage[17] = back_point_x_l;
    parameterpackage[18] = back_point_z_h;
    parameterpackage[19] = back_point_z_l;
    parameterpackage[20] = support_foot_hip_upper_pitch_h;
    parameterpackage[21] = support_foot_hip_upper_pitch_l;
    parameterpackage[22] = kick_foot_ankle_upper_pitch_h;
    parameterpackage[23] = kick_foot_ankle_upper_pitch_l;
    parameterpackage[24] = support_foot_ankle_upper_pitch_h;
    parameterpackage[25] = support_foot_ankle_upper_pitch_l;
    parameterpackage[26] = walking_mode & 0xFF;
    parameterpackage[27] = 0;   //reserve
    parameterpackage[28] = 0;   //reserve
    parameterpackage[29] = 0;   //reserve

    cssl_putdata(serial, parameterpackage, 31);
}

void parameterCallback(const tku_msgs::Parameter_message& msg)
{
    int walking_mode, period_t1, period_t2, sample_time;
    double x_swing, y_swing, z_swing, lock_range;
    double x_swing_com, y_swing_shift, base_default_z, base_lift_z;
    double kick_point_x, kick_point_y, kick_point_z, back_point_x, back_point_z, support_foot_upper_hip_pitch, kick_foot_ankle_upper_pitch,support_foot_upper_ankle_pitch;

    x_swing = msg.X_Swing_Range;
    y_swing = msg.Y_Swing_Range;
    z_swing = msg.Z_Swing_Range;
    period_t1 = msg.Period_T;
    period_t2 = msg.Period_T2;
    sample_time = msg.Sample_Time;
    lock_range = msg.OSC_LockRange;
    base_default_z = msg.BASE_Default_Z;
    y_swing_shift = msg.Y_Swing_Shift;
    x_swing_com = msg.X_Swing_COM;
    base_lift_z = msg.BASE_LIFT_Z;
    walking_mode = msg.Walking_Mode;
    kick_point_x = msg.B_SplineParam.Kick_Point_X;
    kick_point_y = msg.B_SplineParam.Kick_Point_Y;
    kick_point_z = msg.B_SplineParam.Kick_Point_Z;
    back_point_x = msg.B_SplineParam.Back_Point_X;
    back_point_z = msg.B_SplineParam.Back_Point_Z;
    support_foot_upper_hip_pitch = msg.B_SplineParam.Support_Foot_Hip_Upper_Pitch;
    support_foot_upper_ankle_pitch = msg.B_SplineParam.Support_Foot_Ankle_Upper_Pitch;
    kick_foot_ankle_upper_pitch = msg.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch;
    if(walking_mode != 9 && walking_mode != 10)
    {
        paradata_send2fpga(walking_mode, x_swing,  y_swing,  z_swing,  period_t1,  period_t2, sample_time,  lock_range,  base_default_z,  y_swing_shift,  x_swing_com,  base_lift_z);
    }
    else
    {
        paradata_send2fpga(walking_mode, y_swing, period_t1, kick_point_x, kick_point_y, kick_point_z, back_point_x, back_point_z, support_foot_upper_hip_pitch, kick_foot_ankle_upper_pitch, support_foot_upper_ankle_pitch);
    }
}

void walkingdata_send2fpga(int x,int y, int z, int theta, uint8_t walking_cmd, uint8_t sensor_mode)
{
    uint8_t x_h, x_l, y_h, y_l, z_h, z_l, theta_h, theta_l;

    if(x < 0)
    {
        x_h = ((x * -1) >> 8) & 0xFF | 0x80;
        x_l = (x * -1) & 0xFF;
    }
    else
    {
        x_h = (x >> 8) & 0xFF;
        x_l = x & 0xFF;
    }

    if(y < 0)
    {
        y_h = ((y * -1) >> 8) & 0xFF | 0x80;
        y_l = (y * -1) & 0xFF;
    }
    else
    {
        y_h = (y >> 8) & 0xFF;
        y_l = y & 0xFF;
    }

    if(z < 0)
    {
        z_h = ((z * -1) >> 8) & 0xFF | 0x80;
        z_l = (z * -1) & 0xFF;
    }
    else
    {
        z_h = (z >> 8) & 0xFF;
        z_l = z & 0xFF;
    }

    if(theta < 0)
    {
        theta_h = ((theta * -1) >> 8) & 0xFF | 0x80;
        theta_l = (theta * -1) & 0xFF;
    }
    else
    {
        theta_h = (theta >> 8) & 0xFF;
        theta_l = theta & 0xFF;
    }
    
    motorpackage[6] = x_h;
    motorpackage[7] = x_l;
    motorpackage[8] = y_h;
    motorpackage[9] = y_l;
    motorpackage[10] = z_h;
    motorpackage[11] = z_l;
    motorpackage[12] = theta_h;
    motorpackage[13] = theta_l;
    motorpackage[14] = walking_cmd;
    motorpackage[15] = sensor_mode;
    motorpackage[16] = 0;   //reserve
    motorpackage[17] = 0;   //reserve
 
    cssl_putdata(serial, motorpackage, 19);
}

void motionCallback (const tku_msgs::Walking_message& msg)
{
    int x, y, z, theta, walking_cmd, sensor_mode;

    x = msg.X;
    y = msg.Y;
    z = msg.Z;
    theta = msg.THETA;
    walking_cmd = msg.Walking_Cmd;
    sensor_mode = msg.Sensor_Mode;

    walkingdata_send2fpga(x, y, z, theta, walking_cmd, sensor_mode);
}
//---head package---//
void RobotisListini()
{
    RobotisList.clear();
    for(int i = 1; i < 3; i++)
    {
        tsRobotis motor;
        motor.ID = i;
        motor.GoalPosition = 2048;
        motor.Speed = 511;
        RobotisList.push_back(motor);
    }
}
void SYNC_WRITE()
{
    unsigned short blk_size;
    unsigned short crc_value;
///////////////////////////////////////torque///////////////////////////////////////
    // Header
    torquePackage[0] = 0xFF;
    torquePackage[1] = 0xFF;
    torquePackage[2] = 0xFD;
    // Reserved
    torquePackage[3] = 0x00;
    // ID
    torquePackage[4] = 0xFE;
    // Length      The length after the Packet Length field (Instruction, Parameter, CRC fields). Packet Length = number of Parameters + 3
    torquePackage[5] = 0x06;
    torquePackage[6] = 0;
    // Instruction
    torquePackage[7] = 0x03;      //write         // 0x83 = sync write

    // Parameter
    torquePackage[8] = 0x40;      // addressL             // Velocity: 0x70 = 112 Position: 0x74(hex) = 116(dec)
    torquePackage[9] = 0;

    torquePackage[10] = 0x01;      // data length(byte)

    blk_size = 5 + torquePackage[5];
    crc_value = update_crc(0,torquePackage,blk_size);

    torquePackage[11] = (crc_value << 8) >> 8;
    torquePackage[12] = crc_value >> 8;

    for(int cnt = 0; cnt < 13; cnt++)
    {
        cssl_putchar(serial_head,torquePackage[cnt]);
    }
///////////////////////////////////////torque///////////////////////////////////////
    tool->Delay(1);
    // Header
    HeadPackage[0] = 0xFF;
    HeadPackage[1] = 0xFF;
    HeadPackage[2] = 0xFD;
    // Reserved
    HeadPackage[3] = 0x00;
    // ID
    HeadPackage[4] = 0xFE;
    // Length      The length after the Packet Length field (Instruction, Parameter, CRC fields). Packet Length = number of Parameters + 3
    HeadPackage[5] = 0x19;
    HeadPackage[6] = 0;
    // Instruction
    HeadPackage[7] = 0x83;      //write         // 0x83 = sync write
    // Parameter
    HeadPackage[8] = 0x70;      // addressL             // Velocity: 0x70 = 112 Position: 0x74(hex) = 116(dec)
    HeadPackage[9] = 0;

    HeadPackage[10] = 0x08;      // data length(byte)
    HeadPackage[11] = 0;

    HeadPackage[12] = RobotisList[0].ID;      //MotorID
    HeadPackage[13] = RobotisList[0].Speed & 0xFF;
    HeadPackage[14] = (RobotisList[0].Speed >> 8) & 0xFF;
    HeadPackage[15] = (RobotisList[0].Speed >> 16) & 0xFF;
    HeadPackage[16] = (RobotisList[0].Speed >> 24) & 0xFF;
    HeadPackage[17] = RobotisList[0].GoalPosition & 0xFF;
    HeadPackage[18] = (RobotisList[0].GoalPosition >> 8) & 0xFF;
    HeadPackage[19] = (RobotisList[0].GoalPosition >> 16) & 0xFF;
    HeadPackage[20] = (RobotisList[0].GoalPosition >> 24) & 0xFF;

    HeadPackage[21] = RobotisList[1].ID;      //MotorID
    HeadPackage[22] = RobotisList[1].Speed & 0xFF;
    HeadPackage[23] = (RobotisList[1].Speed >> 8) & 0xFF;
    HeadPackage[24] = (RobotisList[1].Speed >> 16) & 0xFF;
    HeadPackage[25] = (RobotisList[1].Speed >> 24) & 0xFF;
    HeadPackage[26] = RobotisList[1].GoalPosition & 0xFF;
    HeadPackage[27] = (RobotisList[1].GoalPosition >> 8) & 0xFF;
    HeadPackage[28] = (RobotisList[1].GoalPosition >> 16) & 0xFF;
    HeadPackage[29] = (RobotisList[1].GoalPosition >> 24) & 0xFF;

    blk_size = 5 + HeadPackage[5];
    crc_value = update_crc(0,HeadPackage,blk_size);

    HeadPackage[30] = (crc_value << 8) >> 8;
    HeadPackage[31] = crc_value >> 8;

    for(int cnt = 0; cnt < 32; cnt++)
    {
        cssl_putchar(serial_head,HeadPackage[cnt]);
    }
}

void HeadMotorFunction(const tku_msgs::HeadPackage &msg)
{
    RobotisList[msg.ID - 1].GoalPosition = msg.Position;
    RobotisList[msg.ID - 1].Speed = msg.Speed;
    SYNC_WRITE();
}
//---head package---//
//---Sector package---//
void Standini()
{
    for(int i =0; i < 87; i++)
    {
        packageMotorData[i] = 0;                            
    } 
    SendSectorPackage.clear();
    printf("Standini\n");
    char pathend[20] = "/sector/";
    char pathend2[20] = "29.ini";
    char path[200];
    int packagecnt;
    int cnt = 3;
    strcpy(path, tool->standPath);
    strcat(path, pathend);
    strcat(path, pathend2);
    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            SendSectorPackage.push_back(tool->readvalue(fin, "Package", 2));
            for(int i = 1; i < packagecnt; i++)
            {
                SendSectorPackage.push_back(tool->readvalue(fin, "|", 3));
            }
        }
        catch(exception e)
        {
        }
        packageMotorData[0] = 0x53;
        packageMotorData[1] = 0x54;
        packageMotorData[2] = 0xF2;
        for(int i =1; i < packagecnt; i++)
        {
            packageMotorData[cnt++] = SendSectorPackage[i];                            
        }      
        cssl_putdata(serial, packageMotorData, cnt);
    }
    printf("End_LoadSector\n");
    SendSectorPackage.clear();
}

void SectorSend2FPGAFunction(const std_msgs::Int16 &msg)
{
    for(int i =0; i < 87; i++)
    {
        packageMotorData[i] = 0;                            
    }
    SendSectorPackage.clear();
    printf("SendSectorPackage\n");
    char filename[10];
    sprintf(filename,"%d",msg.data);
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    char path[200];
    int packagecnt;
    int cnt = 3;
    if(msg.data == 29)
    {
        strcpy(path, tool->standPath);
        strcat(pathend, filename);
        strcat(path, pathend);
        strcat(path, pathend2);
    }
    else
    {
        strcpy(path, tool->parameterPath.c_str());
        strcat(pathend, filename);
        strcat(path, pathend);
        strcat(path, pathend2);
    }
    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            SendSectorPackage.push_back(tool->readvalue(fin, "Package", 2));
            printf("mode = %d\n",SendSectorPackage[0]);
            for(int i = 1; i < packagecnt; i++)
            {
                SendSectorPackage.push_back(tool->readvalue(fin, "|", 3));
            }
        }
        catch(exception e)
        {
        }
        switch(SendSectorPackage[0])
        {
            case 242:
                packageMotorData[0] = 0x53;
                packageMotorData[1] = 0x54;
                packageMotorData[2] = 0xF2;
                for(int i =1; i < packagecnt; i++)
                {
                    packageMotorData[cnt++] = SendSectorPackage[i];                            
                }    
                cssl_putdata(serial, packageMotorData, cnt);
                execute_ack.data = true;
                ExecuteCallBack_Publish.publish(execute_ack);
                printf("Execute is finsih\n");
                break;
            case 243:
                packageMotorData[0] = 0x53;
                packageMotorData[1] = 0x54;
                packageMotorData[2] = 0xF3;
                for(int i =1; i < packagecnt; i++)
                {
                    packageMotorData[cnt++] = SendSectorPackage[i];                       
                }
                cssl_putdata(serial, packageMotorData, cnt);
                execute_ack.data = true;
                ExecuteCallBack_Publish.publish(execute_ack);
                printf("Execute is finsih\n");
                break;
            case 244:
                bool motionlist_flag = true; 
                int cnt_tmp = 1;
                while(motionlist_flag)
                {
                    packageMotorData[0] = 0x53;
                    packageMotorData[1] = 0x54;
                    packageMotorData[2] = 0xF3;
                    for(int i = cnt_tmp; i < packagecnt; i++)
                    {
                        packageMotorData[cnt++] = SendSectorPackage[i];
                        if(SendSectorPackage[i+1] == 68 && SendSectorPackage[i+2] == 89)
                        {
                            cnt_tmp = i + 4;
                            if(SendSectorPackage[cnt_tmp] == 69 && SendSectorPackage[cnt_tmp + 1] == 78)
                            {
                                motionlist_flag = false;
                            }
                            cssl_putdata(serial, packageMotorData, cnt);
							printf("Delay: %d\n",SendSectorPackage[i + 3]);
                            tool->Delay(SendSectorPackage[i + 3]);
                            cnt = 3;
                            break;  
                        }                    
                    }
                }
                execute_ack.data = true;
                ExecuteCallBack_Publish.publish(execute_ack);
                printf("Execute is finsih\n");
                break;
        }
    }
    printf("End_LoadSector\n");
    SendSectorPackage.clear();
}
//---Sector package---//
//---------Interface Save---------//
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{ 
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void InterfaceSaveDataFunction(const tku_msgs::SaveMotion &msg)
{
    MotionSaveData.SaveMotionVector.push_back(msg);
    if(msg.saveflag == true)
    {
        printf("VectorSize = %d\n",MotionSaveData.SaveMotionVector.size()-1);
        string filename = msg.name;
        char pathend[20] = "/";
        char path[200];
        if(msg.savestate == 1)
        {
            strcpy(path, tool->standPath);
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
        }
        else
        {
            strcpy(path, tool->parameterPath.c_str());
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
        }
        ofstream OutFile(path);
        printf("SaveBegin\n");
        OutFile << "VectorCnt = ";
        OutFile << MotionSaveData.SaveMotionVector.size()-1;
        OutFile << "\n";
        for(int i = 0; i < MotionSaveData.SaveMotionVector.size()-1; i++)
        {
            switch(MotionSaveData.SaveMotionVector[i].motionstate)
            {
                case 0:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotionList.size(); j++)
                    {
                        if(j%2 == 0)
                        {
                            OutFile << "A";
                            OutFile << (j/2)+1;
                            OutFile << " = ";
                            OutFile << MotionSaveData.SaveMotionVector[i].MotionList[j];
                            OutFile << "|";
                        }
                        else
                        {
                            OutFile << "D";
                            OutFile << (j/2)+1;
                            OutFile << " = ";
                            OutFile << MotionSaveData.SaveMotionVector[i].MotionList[j];
                            if(j == MotionSaveData.SaveMotionVector[i].MotionList.size()-1)
                            {
                                break;
                            }
                            else
                            {
                                OutFile << "|";
                            }
                        }
                    }
                    OutFile << "\n";
                    break;
                case 1:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 2:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 3:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
                case 4:
                    OutFile << "State = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].motionstate;
                    OutFile << "\n";
                    OutFile << "ID = ";
                    OutFile << MotionSaveData.SaveMotionVector[i].ID;
                    OutFile << "\n";
                    for(int j = 0; j < MotionSaveData.SaveMotionVector[i].MotorData.size(); j++)
                    {
                        OutFile << "M";
                        OutFile << j+1;
                        OutFile << " = ";
                        OutFile << MotionSaveData.SaveMotionVector[i].MotorData[j];
                        if(j == MotionSaveData.SaveMotionVector[i].MotorData.size()-1)
                        {
                            break;
                        }
                        else
                        {
                            OutFile << "|";
                        }
                    }
                    OutFile << "\n";
                    break;
            }               
        }
        printf("SaveEnd\n");
        MotionSaveData.SaveMotionVector.clear();
        OutFile.close();
    }
}
//---------Interface Save---------//
//---------Interface Read---------//
bool InterfaceReadDataFunction(tku_msgs::ReadMotion::Request &Motion_req, tku_msgs::ReadMotion::Response &Motion_res)
{
    printf("LoadParameter\n");
    string filename = Motion_req.name;
    char pathend[20] = "/";
    char path[200];
    char str[10];
    int datacnt;
    int num;
    if(Motion_req.readstate == 1)
    {
        strcpy(path, tool->standPath);
        strcat(pathend, filename.c_str());
        strcat(path, pathend);
    }
    else
    {
        strcpy(path, tool->parameterPath.c_str());
        strcat(pathend, filename.c_str());
        strcat(path, pathend);
    }
    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        try
        {
            printf("Start_Load\n");
            Motion_res.VectorCnt = tool->readvalue(fin, "VectorCnt", 0);
            for(int i = 0; i < Motion_res.VectorCnt; i++)
            {
                Motion_res.motionstate.push_back(tool->readvalue(fin, "State", 0));
                Motion_res.ID.push_back(tool->readvalue(fin, "ID", 0));
                switch(Motion_res.motionstate[i])
                {
                    case 0:
                        for(int j = 0; j < 40; j++)
                        {
                            if((j+1)%2 == 1)
                            {
                                char Motion[20] = "A";
                                sprintf(str,"%d",(j/2)+1);
                                strcat(Motion, str);
                                num = tool->readvalue(fin, Motion, 2);
                            }
                            else
                            {
                                char Delay[20] = "D";
                                sprintf(str,"%d",(j/2)+1);
                                strcat(Delay, str);
                                if(j == 39)
                                {
                                    num = tool->readvalue(fin, Delay, 0);
                                }
                                else
                                {
                                    num = tool->readvalue(fin, Delay, 2);
                                }
                            }
                            Motion_res.MotionList.push_back(num);
                        }
                        break;
                    case 1:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                              num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.RelativeData.push_back(num);
                        }
                        break;
                    case 2:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.RelativeData.push_back(num);
                        }
                        break;
                    case 3:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.AbsoluteData.push_back(num);
                        }
                        break;
                    case 4:
                        for(int j = 0; j < 21; j++)
                        {
                            char Motor[20] = "M";
                            sprintf(str,"%d",j+1);
                            strcat(Motor, str);
                            if(j == 20)
                            {
                                num = tool->readvalue(fin, Motor, 0);
                            }
                            else
                            {
                                num = tool->readvalue(fin, Motor, 2);
                            }
                            Motion_res.AbsoluteData.push_back(num);
                        }
                        break;
                }               

            }
        }
        catch(exception e)
        {
        }
    }
    printf("End_Load\n");
    return true;
}
//---------Interface Read---------//
//---interfaceSendSector package---//
void InterfaceSend2SectorFunction(const tku_msgs::InterfaceSend2Sector &msg)
{
    SaveSectorPackage.push_back(msg.Package);
    int checksum_int = 0;
    int checksum_Lhand_int = 0;
    int checksum_Rhand_int = 0;
    int checksum_Lfoot_int = 0;
    int checksum_Rfoot_int = 0;
    uint8_t checksum_Lhand;
    uint8_t checksum_Rhand;
    uint8_t checksum_Lfoot;
    uint8_t checksum_Rfoot;
    int len = SaveSectorPackage.size();
    if (SaveSectorPackage[0] == 0x53 && SaveSectorPackage[1] == 0x54 && SaveSectorPackage[len-2] == 0x4E && SaveSectorPackage[len-1] == 0x45)
    {
        char pathend[20] = "/sector/";
        char pathend2[20] = ".ini";
        char path[200];
        string filename = msg.sectorname;
        if(filename == "29")
        {
            strcpy(path, tool->standPath);
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
            strcat(path, pathend2);
        }
        else
        {
            strcpy(path, tool->parameterPath.c_str());
            strcat(pathend, filename.c_str());
            strcat(path, pathend);
            strcat(path, pathend2);
        }
        ofstream OutFile(path);
        printf("SaveSectorBegin\n");
        OutFile << "PackageCnt = ";
        OutFile << SaveSectorPackage[len-3];
        OutFile << "\n";
        OutFile << "Package = ";
        OutFile << SaveSectorPackage[2];
        OutFile <<"|| ";
        int pkgsum = 1;
        interface_ack.data = true;
        if(SaveSectorPackage[2] == 242 || SaveSectorPackage[2] == 243)
        {
            if(SaveSectorPackage[len-3] == 85)
            {
                for(int i = 3; i < SaveSectorPackage[len-3] + 2; i++)
                {
                    checksum_int += SaveSectorPackage[i];
                    if(i < 19)
                    {
                        checksum_Lhand_int = checksum_int;
                    }
                    else if(i < 35)
                    {
                        checksum_Rhand_int = checksum_int - checksum_Lhand_int;
                    }
                    else if(i < 63)
                    {
                        checksum_Lfoot_int = checksum_int - checksum_Lhand_int - checksum_Rhand_int;
                    }
                    else
                    {
                        checksum_Rfoot_int = checksum_int - checksum_Lhand_int - checksum_Rhand_int - checksum_Lfoot_int;
                    }
                }
                checksum_Lhand = checksum_Lhand_int & 0xff;
                checksum_Rhand = checksum_Rhand_int & 0xff;
                checksum_Lfoot = checksum_Lfoot_int & 0xff;
                checksum_Rfoot = checksum_Rfoot_int & 0xff;
                printf("checksum_Lhand = %d\n",checksum_Lhand);
                printf("checksum_Rhand = %d\n",checksum_Rhand);
                printf("checksum_Lfoot = %d\n",checksum_Lfoot);
                printf("checksum_Rfoot = %d\n",checksum_Rfoot);
                if(checksum_Lhand == SaveSectorPackage[len-7] && checksum_Rhand == SaveSectorPackage[len-6] && checksum_Lfoot == SaveSectorPackage[len-5] && checksum_Rfoot == SaveSectorPackage[len-4])
                {
                    printf("Send sector is successful!!\n");
                    interface_ack.data = true;
                }
                else
                {
                    printf("Send sector is fail!!\n");
                    interface_ack.data = false;
                }
            }
            else
            {
                printf("Send sector is fail!!\n");
                interface_ack.data = false;
            }
            InterfaceCallBack_Publish.publish(interface_ack);
        }
        for(int i = 3; i < SaveSectorPackage[len-3] + 2; i++)   //[0]&[1] is headpackage so +2 to save last package 
        {
            if(SaveSectorPackage[2] == 244)
            {
                if(SaveSectorPackage[i+1] == 68 && SaveSectorPackage[i+2] == 89)
                {
                    printf("pkgsum = %d\n",pkgsum);
                    if(pkgsum == 84 || pkgsum == 87)
                    {
                        pkgsum = 1;
                        interface_ack.data = true;
                    }
                    else
                    {
                        printf("Send sector is fail!!\n");
                        interface_ack.data = false;
                        InterfaceCallBack_Publish.publish(interface_ack);
                    }
                }
                else
                {
                    if(i == SaveSectorPackage[len-3] + 1)
                    {
                        printf("Send sector is successful!!\n");
                        InterfaceCallBack_Publish.publish(interface_ack);
                    }
                    pkgsum++;
                }
            }
            if(!interface_ack.data)
            {
                break;
            }
            OutFile << SaveSectorPackage[i];
            OutFile <<"|| ";
        }
        printf("SaveSectorEnd\n");
        SaveSectorPackage.clear();
        OutFile.close();
    }
}

bool InterfaceCheckSectorFunction(tku_msgs::CheckSector::Request &req, tku_msgs::CheckSector::Response &res)
{
    CheckSectorPackage.clear();
    printf("CheckSectorStart\n");
    char filename[10];
    sprintf(filename,"%d",req.data);
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    char path[200];
    int packagecnt;
    int returnvalue;
    bool motionlist_flag = true;
    int cnt_tmp = 84;

    if(req.data == 29)
    {
        strcpy(path, tool->standPath);
        strcat(pathend, filename);
        strcat(path, pathend);
        strcat(path, pathend2);
    }
    else
    {
        strcpy(path, tool->parameterPath.c_str());
        strcat(pathend, filename);
        strcat(path, pathend);
        strcat(path, pathend2);
    }
    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        packagecnt = tool->readvalue(fin, "PackageCnt", 0);
        returnvalue = tool->readvalue(fin, "Package", 4);
        if(returnvalue != -1)
        {
            CheckSectorPackage.push_back(returnvalue);
        }
        else
        {
            res.checkflag = false;
            return true;
        }
        printf("mode = %d\n",CheckSectorPackage[0]);
        for(int i = 1; i < packagecnt; i++)
        {
            returnvalue = tool->readvalue(fin, "|", 5);
            if(returnvalue != -1)
            {
                CheckSectorPackage.push_back(returnvalue);
            }
            else
            {
                res.checkflag = false;
                return true;
            }
        }
        switch(CheckSectorPackage[0])
        {
            case 242:
            case 243:
                if(packagecnt != 85)
                {
                    printf("\033[0;31m242 243 Packagecnt is not correct!!\033[0m\n");
                    res.checkflag = false;
                    return true;
                }
                printf("Sector %d is correct!!\n",req.data);
                printf("CheckSectorEnd\n");
                res.checkflag = true;
                return true;
                break;
            case 244:
                while(motionlist_flag)
                {
                    if(cnt_tmp+5 > packagecnt)
                    {
                        printf("\033[0;31m244 count of Package is not the same as Packagecnt!!\033[0m\n");
                        res.checkflag = false;
                        return true;
                    }
                    if(CheckSectorPackage[cnt_tmp+1] == 68 && CheckSectorPackage[cnt_tmp+2] == 89)
                    {
                        if(CheckSectorPackage[cnt_tmp+4] == 69 && CheckSectorPackage[cnt_tmp+5] == 78)
                        {
                            motionlist_flag = false;
                        }
                        cnt_tmp += 87;
                    }
                    else
                    {
                        printf("\033[0;31m244 Package have not 68 89!!\033[0m\n");
                        res.checkflag = false;
                        return true;
                    }
                }
                printf("Sector %d is correct!!\n",req.data);
                printf("CheckSectorEnd\n");
                res.checkflag = true;
                return true;
                break;
            default:
                printf("\033[0;31m%d is not correct mode!!\033[0m\n",SendSectorPackage[0]);
                res.checkflag = false;
                return true;
                break;
        }
    }
}

void MotorSpeedFunction(const tku_msgs::SandHandSpeed &msg)
{
    uint8_t H,L;

    handspeedpackage.clear();

    printf("ReadHandMotionStart\n");
    char filename[10];
    char path[200];
    sprintf(filename,"%d",msg.sector);
    char pathend[20] = "/sector/";
    char pathend2[20] = ".ini";
    strcpy(path, tool->parameterPath.c_str());
    strcat(pathend, filename);
    strcat(path, pathend);
    strcat(path, pathend2);
    int packagecnt;
    int cnt = 3;
    fstream fin;
    fin.open(path, ios::in);
    if(!fin)
    {
        printf("Filename Error!!\n");
    }
    else
    {
        try
        {
            packagecnt = tool->readvalue(fin, "PackageCnt", 0);
            handspeedpackage.push_back(packagecnt);
            handspeedpackage.push_back(tool->readvalue(fin, "Package", 2));
            printf("mode = %d",handspeedpackage[1]);
            for(int i = 1; i < packagecnt; i++)
            {
                handspeedpackage.push_back(tool->readvalue(fin, "|", 3));
            }
        }
        catch(exception e)
        {
        }
    }
    printf("End_LoadSector\n");

    L = msg.speed & 0xFF;
    H = msg.speed>>8 & 0xFF;

    handspeedpackage[18] = L;
    handspeedpackage[19] = H;

    handspeedpackage[117] = L;
    handspeedpackage[118] = H;

    ofstream OutFile(path);
    printf("SendhandspeedBegin\n");
    OutFile << "PackageCnt = ";
    OutFile << handspeedpackage[0];
    OutFile << "\n";
    OutFile << "Package = ";
    for(int i = 1; i < handspeedpackage[0] + 1; i++)   //[0]&[1] is headpackage so +2 to save last package 
    {
        OutFile << handspeedpackage[i];
        printf("%d\n",handspeedpackage[i]);
        OutFile <<"|| ";
    }
    printf("SendhandspeedEnd\n");
    OutFile.close();

    handspeedpackage.clear();
}

void SingleMotorFunction(const tku_msgs::SingleMotorData &msg)
{
    uint8_t Angle_H,Angle_L,Speed_H,Speed_L,checksum = 0;
    int position;
    int cnt = 0;
    position = abs(msg.Position);
    if(msg.Position < 0)
    {
        Angle_L = position & 0xFF;
        Angle_H = (position >>8 & 0xFF)+128;
    }
    else
    {
        Angle_L = position & 0xFF;
        Angle_H = position >>8 & 0xFF;
    }
    Speed_L = msg.Speed & 0xFF;
    Speed_H = msg.Speed>>8 & 0xFF;

    onemotorpackage[0] = 0x53;
    onemotorpackage[1] = 0x54;
    onemotorpackage[2] = 0xF3;

    for(cnt = 3; cnt < (msg.ID - 1) * 4 + 3; cnt++)
    {
        onemotorpackage[cnt] = 0;
    }

    onemotorpackage[cnt++] = Speed_L;
    onemotorpackage[cnt++] = Speed_H;
    onemotorpackage[cnt++] = Angle_L;
    onemotorpackage[cnt++] = Angle_H;

    for(int i = cnt; i < 87; i++)
    {
        onemotorpackage[i] = 0;
    }

    for(int i = 0; i < 87; i++)
    {
        printf("onemotorpackage = %d\n",onemotorpackage[i]);
    }
    cssl_putdata(serial, onemotorpackage, 87);
}
//---HandSpeed package---//

//---Receive & Process Sensor Data 
void Sensor_Data_Process()
{
    uint16_t Sensor_Data_tmp[11];
    double IMU_Value[3];
    uint16_t ForceSensor_Value_tmp[8];
    int ForceSensor_Value[8];    
    int Sensor_Data_Count = 3;
    bool isDataOk = false;
    tku_msgs::SensorPackage sensorpackage;
    std_msgs::Int16 forward_Sector_Number;
    forward_Sector_Number.data  = (short) 78;
    std_msgs::Int16 backward_Sector_Number;
    backward_Sector_Number.data = (short) 87; 

    isBufFull = false;

    for(int i=0; i<3; i++)
    {
        Sensor_Data_tmp[i] = ((sensor_data_buf[Sensor_Data_Count++] << 8) | (sensor_data_buf[Sensor_Data_Count++]));
        if(Sensor_Data_tmp[i] & 0x8000) //negative
        {
            IMU_Value[i] = (double)( ~(Sensor_Data_tmp[i] & 0x7FFF) + 1) / 100.0;
        }
        else                            //positive
        {
            IMU_Value[i] = (double)(Sensor_Data_tmp[i]) / 100.0;
        }
        sensorpackage.IMUData.push_back(IMU_Value[i]);
    }
    
    //For fall down & get up   //Tag for search: #falldown
    /*

    old_fall_Down_Flag = fall_Down_Flag;
    if(sensor_data_buf[Sensor_Data_Count] == 0x46 || sensor_data_buf[Sensor_Data_Count] == 0x42){         
        //(Forward)F =0x46  (Stand)S = 0x53  (Backward)B = 0x42
        fall_Down_Flag = true;
    }
    else if (sensor_data_buf[Sensor_Data_Count] == 0x53)     //(Forward)F =0x46  (Stand)S = 0x53 (Backward)B = 0x42
    {
        fall_Down_Flag = false;
    }
    else
    {
        fall_Down_Flag = fall_Down_Flag;    
    }

 

    //call sector  to  get  up
    if( (old_fall_Down_Flag != fall_Down_Flag) && (fall_Down_Flag == true) && sensor_data_buf[Sensor_Data_Count] == 0x46)//forward fall down
    {
        SectorSend2FPGAFunction(forward_Sector_Number);
        //printf("forward_Sector_Number execute ");
    }
    else if( (old_fall_Down_Flag != fall_Down_Flag) && (fall_Down_Flag == true) && sensor_data_buf[Sensor_Data_Count] == 0x42)//backward fall down
    {
        SectorSend2FPGAFunction(backward_Sector_Number);
        //printf("backward_Sector_Number execute ");
    }
    else if ((old_fall_Down_Flag != fall_Down_Flag && fall_Down_Flag == false))
    {
        //printf("recover to  stand");
        //here  can  add stand flag   if   code  need;
    }
    else
    {
        
    }

    */

    Sensor_Data_Count+=1;

    Sensor_Data_Count+=1;//jump the reserve package byte

    for(int i=0; i<8; i++)
    {
        ForceSensor_Value_tmp[i] = ((sensor_data_buf[Sensor_Data_Count++] << 8) | (sensor_data_buf[Sensor_Data_Count++]));
        if(ForceSensor_Value_tmp[i] & 0x8000)   //negative
        {
            ForceSensor_Value[i] =(((int)((ForceSensor_Value_tmp[i]) & (0x7FFF))) * (-1)) ;
        }
        else                                    //positive
        {
            ForceSensor_Value[i] = (int)(ForceSensor_Value_tmp[i]&0xFFFF) ;
        }   
        sensorpackage.ForceSensorData.push_back(ForceSensor_Value[i]);
    }
    Sensorpackage_Publish.publish(sensorpackage);
    sensorpackage.IMUData.clear();
    sensorpackage.ForceSensorData.clear();
}

void SensorSetFunction(const tku_msgs::SensorSet &msg)
{
    bool Desire_Set = msg.DesireSet;
    bool IMU_Reset = msg.IMUReset;
    bool ForceState = msg.ForceState;
    bool Gain_Set = msg.GainSet;

    if(Desire_Set == true)
    {
        Desire_Roll = msg.Roll;
        Desire_Pitch = msg.Pitch;
        Desire_Yaw = msg.Yaw;
    }
    /*if(Gain_Set == true)
    {
        Gain_Roll = msg.GainRoll;
        Gain_Pitch = msg.GainPitch;
        Gain_KP = msg.GainKP;
        Gain_KD = msg.GainKD;
    }*/

    sensorsetpackage[0] = 0x53;
    sensorsetpackage[1] = 0x54;
    sensorsetpackage[2] = 0xF6;

    if(Desire_Roll < 0)
    {
        Desire_Roll = ~(Desire_Roll) + 1;
        sensorsetpackage[3] = ((Desire_Roll >> 8) & 0xFF) | 0x80;
        sensorsetpackage[4] = Desire_Roll & 0xFF;
        Desire_Roll = ~(Desire_Roll - 1);
    }
    else
    {
        sensorsetpackage[3] = (Desire_Roll >> 8) & 0xFF;
        sensorsetpackage[4] = Desire_Roll & 0xFF;
    }
    if(Desire_Pitch < 0)
    {
        Desire_Pitch = ~(Desire_Pitch) + 1;
        sensorsetpackage[5] = ((Desire_Pitch >> 8) & 0xFF) | 0x80;
        sensorsetpackage[6] = Desire_Pitch & 0xFF;
        Desire_Pitch = ~(Desire_Pitch - 1);
    }
    else
    {
        sensorsetpackage[5] = (Desire_Pitch >> 8) & 0xFF;
        sensorsetpackage[6] = Desire_Pitch & 0xFF;
    }
    if(Desire_Yaw < 0)
    {
        Desire_Yaw = ~(Desire_Yaw) + 1;
        sensorsetpackage[7] = ((Desire_Yaw >> 8) & 0xFF) | 0x80;
        sensorsetpackage[8] = Desire_Yaw & 0xFF;
        Desire_Yaw = ~(Desire_Yaw - 1);
    }
    else
    {
        sensorsetpackage[7] = (Desire_Yaw >> 8) & 0xFF;
        sensorsetpackage[8] = Desire_Yaw & 0xFF;
    }

    sensorsetpackage[9] = (Gain_Set << 3) | (ForceState << 2) | (IMU_Reset << 1) | Desire_Set;
    
    sensorsetpackage[10] = 0;    //Reserve

    sensorsetpackage[11] = 0x45;

    tool->Delay(10);
    cssl_putdata(serial_IMU, sensorsetpackage, SENSOR_SET_PACKAGE_SIZE);
    timer_sensor_set.initialize();
}

void AutoSensorSetFunction()
{
    bool Desire_Set = true;
    bool IMU_Reset = false;
    bool ForceState = false;
    bool Gain_Set = false;

    sensorsetpackage[0] = 0x53;
    sensorsetpackage[1] = 0x54;
    sensorsetpackage[2] = 0xF6;

    if(Desire_Roll < 0)
    {
        Desire_Roll = ~(Desire_Roll) + 1;
        sensorsetpackage[3] = ((Desire_Roll >> 8) & 0xFF) | 0x80;
        sensorsetpackage[4] = Desire_Roll & 0xFF;
        Desire_Roll = ~(Desire_Roll - 1);
    }
    else
    {
        sensorsetpackage[3] = (Desire_Roll >> 8) & 0xFF;
        sensorsetpackage[4] = Desire_Roll & 0xFF;
    }
    if(Desire_Pitch < 0)
    {
        Desire_Pitch = ~(Desire_Pitch) + 1;
        sensorsetpackage[5] = ((Desire_Pitch >> 8) & 0xFF) | 0x80;
        sensorsetpackage[6] = Desire_Pitch & 0xFF;
        Desire_Pitch = ~(Desire_Pitch - 1);
    }
    else
    {
        sensorsetpackage[5] = (Desire_Pitch >> 8) & 0xFF;
        sensorsetpackage[6] = Desire_Pitch & 0xFF;
    }
    if(Desire_Yaw < 0)
    {
        Desire_Yaw = ~(Desire_Yaw) + 1;
        sensorsetpackage[7] = ((Desire_Yaw >> 8) & 0xFF) | 0x80;
        sensorsetpackage[8] = Desire_Yaw & 0xFF;
        Desire_Yaw = ~(Desire_Yaw - 1);
    }
    else
    {
        sensorsetpackage[7] = (Desire_Yaw >> 8) & 0xFF;
        sensorsetpackage[8] = Desire_Yaw & 0xFF;
    }

    sensorsetpackage[9] = (Gain_Set << 3) | (ForceState << 2) | (IMU_Reset << 1) | Desire_Set;
    
    sensorsetpackage[10] = 0;    //Reserve


    sensorsetpackage[11] = 0x45;

    if(timer_sensor_set.checkTimePass())
    {
        cssl_putdata(serial_IMU, sensorsetpackage, SENSOR_SET_PACKAGE_SIZE);
        timer_sensor_set.initialize();
    }
}

/*==============================================================================*/
//Main
/*==============================================================================*/
int main(int argc, char **argv)
{
	//Initial
    RobotisListini();
    packageinit();

    ros::init(argc, argv, "motionpackage");
	ros::NodeHandle nh;

    ros::Subscriber parameter_sub = nh.subscribe("/package/parameterdata", 1000, parameterCallback);
    ros::Subscriber motion_sub = nh.subscribe("/package/walkingdata", 1000, motionCallback);
    ros::ServiceServer InterfaceReadData_service = nh.advertiseService("/package/InterfaceReadSaveMotion",InterfaceReadDataFunction);
    ros::ServiceServer InterfaceCheckSector_service = nh.advertiseService("/package/InterfaceCheckSector",InterfaceCheckSectorFunction);
    
    ros::Subscriber headmotor_subscribe = nh.subscribe("/package/HeadMotor", 1000, HeadMotorFunction);
    ros::Subscriber SectorSend2FPGA_subscribe = nh.subscribe("/package/Sector", 1000, SectorSend2FPGAFunction);
    ros::Subscriber motorspeed_subscribe = nh.subscribe("/package/motorspeed", 1000, MotorSpeedFunction);
    ros::Subscriber SingleMotorData_subscribe = nh.subscribe("/package/SingleMotorData", 1000, SingleMotorFunction);
    ros::Subscriber InterfaceSend2Sector = nh.subscribe("/package/InterfaceSend2Sector", 1000, InterfaceSend2SectorFunction);
    ros::Subscriber InterfaceSaveData_Subscribe = nh.subscribe("/package/InterfaceSaveMotion", 1000, InterfaceSaveDataFunction);
    ros::Subscriber SensorSet_Subscribe = nh.subscribe("/sensorset", 100, SensorSetFunction);
    
    FPGAack_Publish = nh.advertise<std_msgs::Int16>("/package/FPGAack", 1000);
    walkack_Publish = nh.advertise<std_msgs::Bool>("/package/walkack", 1000);
    InterfaceCallBack_Publish = nh.advertise<std_msgs::Bool>("/package/motioncallback", 1000);
    ExecuteCallBack_Publish = nh.advertise<std_msgs::Bool>("/package/executecallback", 1000);
    Sensorpackage_Publish = nh.advertise<tku_msgs::SensorPackage>("/package/sensorpackage", 1000);
    
    gettimeofday(&tstart, NULL);
    do{

        if(mcssl_init() > 0)
        {
            Standini();
            break;
        }else
        {
            usleep(1000000);//1s = 1,000,000 us
        }

    }while(ros::ok());
    printf("Motion is running\n");
    usleep(1000000);
    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        if(read_IMU_count >= 3)// 60/4 = 15Hz
        {
            AutoSensorSetFunction();
            read_IMU_count = 0;
        }
        else
        {
            read_IMU_count++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    mcssl_finish();

    return 0;
}
