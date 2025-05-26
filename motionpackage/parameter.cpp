/**
 * @file  src/qt_ros_pkg/ClassInfo/classinfo.cpp
 *
 * @brief Bridge between Ros with Task
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "walkinggait/parameterinfo.h"

ParameterInfo* parameterinfo = new ParameterInfo();
/*****************************************************************************
** Implementation
*****************************************************************************/
Parameters::Parameters()
{
    this->X_Swing_Range = 0;
    this->Y_Swing_Range = 0;
    this->Z_Swing_Range = 0;
    this->Y_Swing_Shift = 0;
    this->Period_T = 1200;
    this->Period_T2 = 0;
    this->Period_COM_Y = 0;
    this->Sample_Time = 12;
    this->OSC_LockRange = 0;
    this->BASE_Default_Z = 0;
    this->Ts = 150;
    this->Vmax = 0.1;
    this->Vmin = 0.09;
    this->COM_Height = 17.9;
    this->abswaistx = 0;
    this->Stand_Balance = false;

    this->X_Swing_COM = 0;
    this->BASE_LIFT_Z = 0;

    this->Kick_Point_X = 0;
    this->Kick_Point_Y = 0;
    this->Kick_Point_Z = 0;
    this->Back_Point_X = 0;
    this->Back_Point_Z = 0;
    this->Support_Foot_Hip_Upper_Pitch = 0;
    this->Kick_Foot_Ankle_Upper_Pitch = 0;
    this->Support_Foot_Ankle_Upper_Pitch = 0;

    this->rightfoot_shift_z = 0;
    this->com_y_swing = 0;
    this->now_stand_height = 23.5;
    this->now_com_height = 29.5;
}

Parameters::~Parameters()
{

}
Points::Points()
{
    wv_x_COM = 0;
    abs_x_feet = 0;
    foot_x_r = 0;
    foot_x_l = 0;
}

Points::~Points()
{

}

COMPlan::COMPlan()
{
    this->sample_point_ = 0;
    this->time_point_ = 0;
    this->walking_state = StopStep;
    this->walking_stop = true;//false
    this->lift_lock_x = 0;
    this->lift_lock_z = 0;
    this->isLfootfirst = false;
}

COMPlan::~COMPlan()
{

}

ParameterInfo::ParameterInfo()
{
    this->rosflag = false;
    this->doflag = false;
    this->walkingpointflag = false;
    this->X = 0;
    this->Y = 0;
    this->Z = 0;
    this->THETA = 0;

    this->hand = 0;

    this->walking_mode = 0;
    this->walking_state = 0;
    this->sensor_mode = 0;

    this->XUpdate = 0;
    this->YUpdate = 0;
    this->ZUpdate = 0;
    this->THETAUpdate = 0;

    this->WalkFlag = false;
    this->IsParametersLoad = false;
    this->FpgaFlag = false;

    this->serialack = 0;
    this->cpgack = false;
    this->Repeat = false;

    this->control = 0;
    this->FPGAState = etReady;
    this->CPGalready = false;

    this->NowStep.data = 0;
    this->Isfrist_right_shift = true;
}

ParameterInfo::~ParameterInfo()
{

}


IK_Parameters::IK_Parameters()
{

}

IK_Parameters::~IK_Parameters()
{

}
