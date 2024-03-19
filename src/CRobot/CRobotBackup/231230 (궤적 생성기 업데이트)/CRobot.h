/*
 * Control Robot Class
 * 
 * Robotics & Control Lab.
 * 
 * File:   CRobot.h
 * Author: BK Cho
 *
 * First developer : SungJoon Yoon
 * 
 * ======
 * Update date : 2023.03.14 by SungJoon Yoon
 * ======
 *
 */
 
#ifndef CROBOT_H
#define CROBOT_H

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL

//* Header file for OSQP
#include "osqp.h"

//* Basic header, Library
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>

//* Header for using nrutil library
#include "nrutil/nrlin.h"


#define PI              3.14159265359
#define PI2             6.28318530718
#define D2R             PI/180.
#define R2D             180./PI

#define GRAVITY         9.81
#define JOINT_NUM       12
#define AXIS_X          0
#define AXIS_Y          1
#define AXIS_Z          2

#define AXIS_ROLL       0
#define AXIS_PITCH      1
#define AXIS_YAW        2

#define AXIS_QUAT_X     0
#define AXIS_QUAT_Y     1
#define AXIS_QUAT_Z     2
#define AXIS_QUAT_W     3

#define GEAR_RATIO 50

//#define tasktime 0.002
//#define onesecSize 500
#define onesecScale 1000

//Print color
#define C_BLACK         "\033[30m"
#define C_RED           "\033[0;31m"
#define C_GREEN         "\033[32m"
#define C_YELLOW        "\033[33m"
#define C_BLUE          "\033[0;34m"
#define C_PURPLE        "\033[35m"
#define C_MAGENTA       "\033[95m"
#define C_CYAN          "\033[36m"
#define C_LIGHTGRAY     "\033[37m"
#define C_DARKGRAY      "\033[1;30m"
#define C_RESET         "\033[0m"
        


//** WsJi, MPC: Horizontal length... 이 선언 위치를 바꾸는 게 좋을텐데 아직은 고민이다..
#define horizontal_length   9
#define mpc_sampletime 0.040
// #define mpc_sampletime 0.020

// 새로 사용중인 시간간격?
#define horizontal_N   9


using namespace std;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;

typedef Eigen::Matrix<float, JOINT_NUM, 1> Vector12f;
typedef Eigen::Matrix<float, JOINT_NUM+6, 1> Vector18f;
typedef Eigen::Matrix<float, JOINT_NUM+7, 1> Vector19f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef enum {
    SIMULATION,
    ACTUAL_ROBOT
} _MODE;

typedef enum {
    LEGMODE,
    WHEELMODE
} _TYPE;

typedef enum {
    CTRLMODE_NONE,
    CTRLMODE_HOME_POSE,
    CTRLMODE_WALK_READY,
    CTRLMODE_TROT,
    CTRLMODE_JUMP,
} _CONTROL_MODE;

typedef enum {
    NONE_ACT,
    HOME_POSE,
    WALK_READY,
    TROT,
    FLYING_TROT,
    TORQUE_OFF,
    STAIR,
    PRONK,
    STAND,
} _COMMAND_FLAG;

typedef enum {
    CONTACT_OFF,
    CONTACT_ON

} _CONTACT_INFO;

typedef enum {
    PRONK_NONE,
    PRONK_JUMP_READY,
    PRONK_TAKE_OFF,
    PRONK_SWING,
    PRONK_LAND_TAKE_OFF,
    PRONK_LAND,
    PRONK_FINAL
} _JUMP_PHASE;

typedef enum {
    FT_NONE,
    FT_READY,
    FT_STANCE_RLFR,
    FT_SWING_RLFR,
    FT_STANCE_RRFL,
    FT_SWING_RRFL,
    FT_LAND,
    FT_FINAL
} _FT_PHASE;

typedef enum {
    TROT_NONE,
    TROT_READY,
    TROT_STANCE,
    TROT_SWING,
} _TROT_PHASE;

typedef enum {
    STAIR_NONE,
    STAIR_READY,
    STAIR_STANCE,
    STAIR_SWING,
    STAIR_STANCE_RLFR,
    STAIR_SWING_RLFR,
    STAIR_STANCE_RRFL,
    STAIR_SWING_RRFL,
    STAIR_FINAL_STANCE,
    STAIR_FINAL_SWING,
} _STAIR_PHASE;


//**
typedef enum {
    LEGPHASE_NONE,
    LEGPHASE_FL,
    LEGPHASE_RR,
    LEGPHASE_FR,
    LEGPHASE_RL,
    LEGPHASE_FLRR,
    LEGPHASE_FRRL,
} _LEG_PHASE;

typedef enum {
    MOVE_WALK,
    MOVE_TROT,
}_MOVE_MODE;

typedef enum{
    STANCE_PHASE,
    SWING_PHASE,
}_GAIT_PHASE;

//**

typedef struct Base {
    //* Current information (Global Frame)
    Vector3f G_CurrentPos;                                          // EndPoint's Current Position                     in Global frame coordinate system
    Vector3f G_CurrrentVel;                                         // EndPoint's Current Velocity                     in Global frame coordinate system
    Vector3f G_CurrentAcc;                                          // EndPoint's Current Acceleration                 in Global frame coordinate system
                             
    Vector3f G_CurrentPrePos;                                       // EndPoint's Current PrePosition                  in Global frame coordinate system
    Vector3f G_CurrentPreVel;                                       // EndPoint's Current PreVelocity                  in Global frame coordinate system
    Vector3f G_CurrentPreAcc;                                       // EndPoint's Current PreAcceleration              in Global frame coordinate system
                          
    Vector3f G_CurrentOri;                                          // EndPoint's Current Orientation(EuleZYX)         in Global frame coordinate system
    Vector4d G_CurrentQuat;                                         // EndPoint's Current Orientation(Quaternion)      in Global frame coordinate system
    Vector3f G_CurrentAngularVel;                                   // EndPoint's Current AngularVelocity              in Global frame coordinate system
    Vector3f G_CurrentAngularAcc;                                   // EndPoint's Current AngularAcceleration          in Global frame coordinate system
   
    Vector3f G_Foot_to_Base_CurrentPos;
    Vector3f G_Foot_to_Base_CurrentVel;
    
    //* Reference information (Global Frame)
    Vector3f G_RefPos;
    Vector3f G_RefVel;
    Vector3f G_RefAcc;
   
    Vector3f G_RefPrePos;
    Vector3f G_RefPreVel;
    Vector3f G_RefPreAcc;
    
    Vector3f G_RefOri;                                              // EndPoint's Reference Orientation                in Global frame coordinate system
    Vector4d G_RefQuat;                                             // EndPoint's Reference Orientation(Quaternion)    in Global frame coordinate system
    Vector3f G_RefAngularVel;                                       // EndPoint's Reference AngularVelocity            in Global frame coordinate system
    Vector3f G_RefAngularAcc;                                       // EndPoint's Reference AngularAcceleration        in Global frame coordinate system
    
    Vector3f G_Foot2CoM_RefPos;
    Vector3f G_Foot2CoM_RefVel;
    
    Vector3f G_RefPreOri;                                           // EndPoint's Reference Orientation                in Global frame coordinate system
    Vector3f G_RefPreAngularVel;                                    // EndPoint's Reference AngularVelocity            in Global frame coordinate system
    Vector3f G_RefPreAngularAcc;                                    // EndPoint's Reference AngularAcceleration        in Global frame coordinate system
   
    Vector3f G_RefTargetPos;                                        // EndPoint's Reference TargetPosition             in Global frame coordinate system
    Vector3f G_RefTargetVel;                                        // EndPoint's Reference TargetVelocity             in Global frame coordinate system
    Vector3f G_RefTargetAcc;                                        // EndPoint's Reference TargetAcceleration         in Global frame coordinate system
    
    Vector4d G_RefTargetQuat;   // 안쓰는 걸로 보임
    
    Vector3f G_Foot_to_Base_RefPos;
    Vector3f G_Foot_to_Base_RefVel;
    
    //* Initial information
    Vector3f G_InitPos;
    Vector3f G_InitVel;
    Vector3f G_InitAcc;

    Vector3f G_InitOri;
    Vector3f G_InitAngularVel;
    Vector3f G_InitAngularAcc;
    Vector4d G_InitQuat;
    
    // Reference Target Orientation, AngularVelocity, AngularAcceleration
    Vector3f G_RefTargetOri;
    Vector3f G_RefTargetOri_dot                 = Vector3f::Zero(3);
    Vector3f G_RefTargetAngularVel              = Vector3f::Zero(3);
    Vector3f G_RefTargetAngularAcc              = Vector3f::Zero(3);
    
    //* Current information (Base Frame)
    Vector3f B_RefAngularVel;                                       
    Vector3f B_RefAngularAcc;
    
    Vector3f B_RefPreAngularVel; 

    Vector3f B_CurrentAngularVel;                                   
    
    //* GroundTruth
    Vector3f G_GroundTruthPos;
    Vector3f G_GroundTruthVel;
    Vector3f G_GroundTruthOri;
    Vector4d G_GroundTruthQuat;
    
    Matrix3f TargetRotMat;
    Matrix3f CurrentRotMat;
    Matrix3f RefRotMat;
    
    //* 5thPolyNomial Coefficient
    Vector6f Roll_Coeff      = Vector6f::Zero(6);
    Vector6f Pitch_Coeff     = Vector6f::Zero(6);
    Vector6f Yaw_Coeff       = Vector6f::Zero(6);
    
    Vector3f Turnsize;
    
    int ID;                                                         // ID of each link at URDF file
    
} BASE;

typedef struct CoM {
    float Height;

    //* Current information    
    Vector3f G_CurrentPos;                                          // CoM's Current Position                     in Global frame coordinate system
    Vector3f G_CurrrentVel;                                         // CoM's Current Velocity                     in Global frame coordinate system
    Vector3f G_CurrentAcc;                                          // CoM's Current Acceleration                 in Global frame coordinate system
                             
    Vector3f G_CurrentPrePos;                                       // CoM's Current PrePosition                  in Global frame coordinate system
    Vector3f G_CurrentPreVel;                                       // CoM's Current PreVelocity                  in Global frame coordinate system
    Vector3f G_CurrentPreAcc;                                       // CoM's Current PreAcceleration              in Global frame coordinate system
                          
    Vector3f G_CurrentOri;                                          // CoM's Current Orientation(EuleZYX)         in Global frame coordinate system
    Vector4d G_CurrentQuat;                                         // CoM's Current Orientation(Quaternion)      in Global frame coordinate system
    Vector3f G_CurrentAngularVel;                                   // CoM's Current AngularVelocity              in Global frame coordinate system
    Vector3f G_CurrentAngularAcc;                                   // CoM's Current AngularAcceleration          in Global frame coordinate system
   
    Vector3f G_Foot_to_CoM_CurrentPos;
    Vector3f G_Foot_to_CoM_CurrentVel;
    Vector3f G_Foot_to_CoM_CurrentPrePos;
    Vector3f G_Foot_to_CoM_CurrentPreVel;
    
    //* CoM & Base Offset(Position)
    Vector3f G_Base_to_CoM_CurrentPos;
    Vector3f B_Base_to_CoM_CurrentPos;
    
    Vector3f G_CoM_to_Base_CurrentPos;
    Vector3f B_CoM_to_Base_CurrentPos;
    
    
    //* CoM & Base Offset(Velocity)
    Vector3f G_Base_to_CoM_CurrentVel;
    Vector3f B_Base_to_CoM_CurrentVel;
    
    Vector3f G_CoM_to_Base_CurrentVel;
    Vector3f B_CoM_to_Base_CurrentVel;
    
    //
    Vector3f B_CurrentPos;                                          // CoM's Current Position                     in Global frame coordinate system
    Vector3f B_CurrrentVel;                                         // CoM's Current Velocity                     in Global frame coordinate system
    Vector3f B_CurrentAcc;                                          // CoM's Current Acceleration                 in Global frame coordinate system
                             
    Vector3f B_CurrentPrePos;                                       // CoM's Current PrePosition                  in Global frame coordinate system
    Vector3f B_CurrentPreVel;                                       // CoM's Current PreVelocity                  in Global frame coordinate system
    Vector3f B_CurrentPreAcc;                                       // CoM's Current PreAcceleration              in Global frame coordinate system    
    
    // Reference information    
    Vector3f G_RefPos;
    Vector3f G_RefVel;
    Vector3f G_RefAcc;

    Vector3f G_RefPrePos;
    Vector3f G_RefPreVel;
    Vector3f G_RefPreAcc;
    
    Vector3f G_RefOri;                                              // EndPoint's Reference Orientation                in Global frame coordinate system
    Vector4d G_RefQuat;                                             // EndPoint's Reference Orientation(Quaternion)    in Global frame coordinate system
    Vector3f G_RefAngularVel;                                       // EndPoint's Reference AngularVelocity            in Global frame coordinate system
    Vector3f G_RefAngularAcc;                                       // EndPoint's Reference AngularAcceleration        in Global frame coordinate system
    
    Vector3f G_Foot_to_CoM_RefPos;
    Vector3f G_Foot_to_CoM_RefVel;
    Vector3f G_Foot_to_CoM_RefPrePos;
    Vector3f G_Foot_to_CoM_RefPreVel;
    
    //* CoM & Base Offset(Position)
    Vector3f G_Base_to_CoM_RefPos;
    Vector3f B_Base_to_CoM_RefPos;
    Vector3f G_CoM_to_Base_RefPos;
    Vector3f B_CoM_to_Base_RefPos;
    
    //* CoM & Base Offset(Velocity)
    Vector3f G_Base_to_CoM_RefVel;
    Vector3f B_Base_to_CoM_RefVel;
    Vector3f G_CoM_to_Base_RefVel;
    Vector3f B_CoM_to_Base_RefVel;
   
    //
    Vector3f G_RefTargetPos;                                        // EndPoint's Reference TargetPosition             in Global frame coordinate system
    Vector3f G_RefTargetVel;                                        // EndPoint's Reference TargetVelocity             in Global frame coordinate system
    Vector3f G_RefTargetAcc;                                        // EndPoint's Reference TargetAcceleration         in Global frame coordinate system
    
    //* Initial information
    Vector3f G_InitPos;
    Vector3f G_InitVel;
    Vector3f G_InitAcc;
            
    //* 5thPolyNomial Coefficient    
    Vector6f X_Coeff       = Vector6f::Zero(6);
    Vector6f Y_Coeff       = Vector6f::Zero(6);
    Vector6f Z_Coeff       = Vector6f::Zero(6);
    
    //* StepSize
    Vector3f MoveSize;
    
} COM;

typedef struct EndPoint {
    //* Current information (Global)
    Vector3f G_CurrentPos;                                          // EndPoint's Current Position in Global frame coordinate system
    Vector3f G_CurrentVel;                                          // EndPoint's Current Velocity in Global frame coordinate system
    Vector3f G_CurrentAcc;                                          // EndPoint's Current Acceleration in Global frame coordinate system
    
    Vector3f G_CurrentPrePos;                                       // EndPoint's Current PrePosition in Global frame coordinate system
    Vector3f G_CurrentPreVel;                                       // EndPoint's Current PreVelocity in Global frame coordinate system
    Vector3f G_CurrentPreAcc;                                       // EndPoint's Current PreAcceleration in Global frame coordinate system
    
    Vector3f G_CoM_to_Foot_CurrentPos;
    Vector3f G_CoM_to_Foot_CurrentVel;
    Vector3f G_CoM_to_Foot_CurrentAcc;
    
    Vector3f G_Base_to_Foot_CurrentPos;
    Vector3f G_Base_to_Foot_CurrentVel;
    Vector3f G_Base_to_Foot_CurrentAcc;
    
    //* Reference information (Global)
    Vector3f G_RefPos;
    Vector3f G_RefVel;
    Vector3f G_RefAcc;

    Vector3f G_RefPrePos;
    Vector3f G_RefPreVel;
    Vector3f G_RefPreAcc;
    
    Vector3f G_CoM_to_Foot_RefPos;
    Vector3f G_CoM_to_Foot_RefVel;
    Vector3f G_CoM_to_Foot_RefAcc;
    
    Vector3f G_Base_to_Foot_RefPos;
    Vector3f G_Base_to_Foot_RefVel;
    Vector3f G_Base_to_Foot_RefAcc;
    
    Vector3f G_RefTargetPos;
    Vector3f G_RefTargetVel;
    Vector3f G_RefTargetAcc;
    
    float* G_Base2Foot_RefPos                = fvector(1, 3);   // 
    float* G_Base2Foot_RefVel                = fvector(1, 3);   // 
    float* G_CoM2Foot_RefPos                 = fvector(1, 3);
    float* G_CoM2Foot_RefVel                 = fvector(1, 3);    
            
    float* B_Base2Foot_RefPos                = fvector(1, 3);
    float* B_Base2Foot_RefVel                = fvector(1, 3);
    float* B_CoM2Foot_RefPos                 = fvector(1, 3);
    float* B_CoM2Foot_RefVel                 = fvector(1, 3);    
    
    float* G_Base2Foot_CurrentPos            = fvector(1, 3);
    float* G_Base2Foot_CurrentVel            = fvector(1, 3);
    float* G_CoM2Foot_CurrentPos             = fvector(1, 3);
    float* G_CoM2Foot_CurrentVel             = fvector(1, 3);    
        
    float* B_Base2Foot_CurrentPos            = fvector(1, 3);
    float* B_Base2Foot_CurrentVel            = fvector(1, 3);
    float* B_CoM2Foot_CurrentPos             = fvector(1, 3);
    float* B_CoM2Foot_CurrentVel             = fvector(1, 3);

    float* tmp_B_Base2Foot_CurrentPos        = fvector(1, 3);    
    
    
    //* Initial information
    Vector3f G_InitPos;
    Vector3f G_InitVel;
    Vector3f G_InitAcc;
    
    //* Reference information (Base)
    Vector3f B_RefPos;
    Vector3f B_RefVel;
    Vector3f B_RefAcc;
        
    Vector3f B_CoM_to_Foot_RefPos;
    Vector3f B_CoM_to_Foot_RefVel;
    Vector3f B_CoM_to_Foot_RefAcc;
    
    //* Current information (Base)
    Vector3f B_CoM_to_Foot_CurrentPos;
    Vector3f B_CoM_to_Foot_CurrentVel;
    Vector3f B_CoM_to_Foot_CurrentAcc;
    
    Vector3f B_CurrentPos;                                          // EndPoint's Current Position in Global frame coordinate system
    Vector3f B_CurrentPos2;                                          // EndPoint's Current Position in Global frame coordinate system
    Vector3f B_CurrentVel;                                          // EndPoint's Current Velocity in Global frame coordinate system
    Vector3f B_CurrentAcc;                                          // EndPoint's Current Acceleration in Global frame coordinate system
        
    //* GroundTruth
    Vector3f G_GroundTruthPos;
    
    //* ControlPoint
    VectorXf ControlPointX       = VectorXf::Zero(9);
    VectorXf ControlPointY       = VectorXf::Zero(9);
    VectorXf ControlPointZ       = VectorXf::Zero(9);
    
    // FootStep
    Vector3f MoveSize;
    Vector3f TurnSize;
            
    int ID;
        
    Vector6f X_Coeff            = Vector6f::Zero(6);
    Vector6f Y_Coeff            = Vector6f::Zero(6);
    
    Vector6f Z_Coeff            = Vector6f::Zero(6);
    
    Vector6f Z_Coeff_Up         = Vector6f::Zero(6);
    Vector6f Z_Coeff_Down       = Vector6f::Zero(6);
    
    Vector3f FTSensor;
    
    Vector2d distTorque         = VectorXd::Zero(2);
    Vector2d distTorque_estimated = VectorXd::Zero(2);
    Vector2d temp_now = VectorXd::Zero(2);
    Vector2d temp_filtered = VectorXd::Zero(2);

    Vector3d GRF_estimated = VectorXd::Zero(3);
    MatrixXd Jacobian_2d = MatrixXd::Zero(2,2);
    // MatrixXd Jacobian_Ttemp = MatrixXd::Zero(2,3);
    MatrixXd Jacobian_2d_T = MatrixXd::Zero(2,2);
    MatrixXd Jacobian_2d_T_inv = MatrixXd::Zero(2, 2);
    MatrixXd Jacobian_2d_T_inv_use = MatrixXd::Zero(3, 2);


} ENDPOINT;

typedef struct Joint {
    //* Initial information
    float InitPos;
    float InitVel;
    float InitAcc;

    //* Reference information
    float RefPos;
    float RefVel;
    float RefAcc;
    
    float RefPrePos;
    float RefPreVel;
    float RefPreAcc;
    
    //* Reference Torque
    float RefTorque;
    
    //* Target information
    float RefTargetPos;
    float RefTargetVel;
    float RefTargetAcc;

    //* Current information   
    float CurrentPos;
    float CurrentVel;
    float CurrentAcc;
    
    float CurrentPrePos;
    float CurrentPreVel;
    float CurrentPreAcc;
    
    float Incremental_CurrentPos;
    float Incremental_CurrentPosOffset;
    float Absolute_CurrentPos;

    //* Current Torque
    float CurrentTorque;

    //* Joint Gain
    float KpGain;
    float KdGain;
    
} JOINT;


typedef struct Wheel {
    
    float Velocity;
    float SpeedWeight;
    
} WHEEL;


typedef struct BodyKinematic {
    float TargetCoMHeight;
    float BaseHeight;
    
    // Mass
    float m_total;
    
    float m_body_link;
    float m_FL_hip_link, m_FL_thigh_link, m_FL_calf_link, m_FL_tip_link;
    float m_FR_hip_link, m_FR_thigh_link, m_FR_calf_link, m_FR_tip_link;
    float m_RL_hip_link, m_RL_thigh_link, m_RL_calf_link, m_RL_tip_link;
    float m_RR_hip_link, m_RR_thigh_link, m_RR_calf_link, m_RR_tip_link;
    
    int i_body_link;
    int i_FL_hip_link, i_FL_thigh_link, i_FL_calf_link, i_FL_tip_link;
    int i_FR_hip_link, i_FR_thigh_link, i_FR_calf_link, i_FR_tip_link;
    int i_RL_hip_link, i_RL_thigh_link, i_RL_calf_link, i_RL_tip_link;
    int i_RR_hip_link, i_RR_thigh_link, i_RR_calf_link, i_RR_tip_link;
    
    Vector3f c_body_link;
    Vector3f c_FL_hip_link, c_FL_thigh_link, c_FL_calf_link, c_FL_tip_link;
    Vector3f c_FR_hip_link, c_FR_thigh_link, c_FR_calf_link, c_FR_tip_link;
    Vector3f c_RL_hip_link, c_RL_thigh_link, c_RL_calf_link, c_RL_tip_link;
    Vector3f c_RR_hip_link, c_RR_thigh_link, c_RR_calf_link, c_RR_tip_link;
    
    Matrix3f B_I_g;
    Matrix3f G_I_g;
    float** B_I_tensor       = matrix(1, 3, 1, 3);            // B_I_g is the inertia tensor in Body coordinates.
    float** G_I_tensor       = matrix(1, 3, 1, 3);            // G_I_g is the inertia tensor in Global coordinates.
    
    
} BodyKinematic;


typedef struct WalkingTime {
    float sec;
    float ready;
    float stance;
    float swing;
    float step;

} WALKINGTIME;

typedef struct Step {
    float footHeight;
    
    BASE base;
    COM com;
    ENDPOINT FL_Foot, FR_Foot, RL_Foot, RR_Foot;
    
    float count;

    // Phase
    float RLFR_SWING_START;
    float RLFR_STANCE_START;
    float RRFL_SWING_START;
    float RRFL_STANCE_START;

    float FINAL_RLFR_SWING_START;
    float FINAL_RLFR_STANCE_START;
    float FINAL_RRFL_SWING_START;
    float FINAL_RRFL_STANCE_START;
    float FINAL_RRFL_STANCE_END;

    // FootStep
    float FBstepSize;
    float LRstepSize;
    float TurnSize;
    float FootHeight;
    
    enum {
        LeftTurn    =   1,
        NoTurn      =   0,
        RightTurn   =  -1,
        TurnModeOn  =   1,
        TurnModeOff =   0,
    };
    

    
    int  walking_TurnDirection;
    int  walking_Turn;
    float turningPoint;
    enum {
        START_FOOT_RLFR    =  1,
        START_FOOT_RRFL    = -1,
        NONE               =  0,
    };

    int start_foot;

    //** 추가해줌
    int start_foot_prev   = 0;

} STEP;

typedef struct TrotWalk {
    Vector3f SpeedScale;
    Vector3f LimitAcceleration;
    
    float ActiveThreshold;
    
} TROTWALK;

typedef struct Walking {
    STEP step;
    TROTWALK trot;
    WALKINGTIME time;
    bool UpDown;
    bool Ready;
    bool MoveDone;
    bool HomePoseDone;
    bool Stop;
    int phase;

    Vector3f InputData;
    Vector3f CoM_InputData              = Vector3f::Ones(3);
    Vector3f MovingSpeed;
    Vector3f PreMovingSpeed;

    Vector3f PreState;
    Vector3f TargetState;
    
    Vector3f PreState_Up;
    Vector3f PreState_Down;
    Vector3f TargetState_Up;
    Vector3f TargetState_Down;
} WALK;

// typedef struct WalkingPattern{
//     // CoM (for supporting motion)
//     Vector3f CoM_pos = VectorXf::Zero(3);

//     // Flag
//     bool walkingFlag = false;
//     unsigned int 
//     // 

//     // Each legs
//     // RL
//     Vector3f RL_pos = VectorXf::Zero(3);
//     Vector3f RL_swing_pos = VectorXf::Zero(3);
//     Vector3f RL_swing_prev = VectorXf::Zero(3);
//     Vector3f RL_support_pos = VectorXf::Zero(3);    // = -CoM_pos
//     Vector3f RL_walk_temp = VectorXf::Zero(3);
//     Vector3f RL_offset = VectorXf::Zero(3);
//     Vector3f RL_pos_with_offset = VectorXf::Zero(3);

//     // Joystick
//     Vector3f vel_joy = VectorXf::Zero(3);

//     // SimTime
//     float pdt = 0.;  // time in phase
//     float walkingdt = 0.;    // time in 4-phase ( = 1 walking phase )


// } WALKINGPATTERN;





typedef struct StandTurn {
    bool Startflag                      = true;
    VectorXf Activeflag_XYZ             = VectorXf::Zero(6);
    VectorXf Activeflag_RPY             = VectorXf::Zero(6);
    
    float time;
    
    Vector3f G_InitOri;
    Vector3f G_RefPreOri;
    Vector3f G_RefTargetOri;

    Vector3f G_RefOri;
    Vector3f G_RefOri_dot;
    Vector3f G_RefAngularVel;
    
    Vector3f G_InitPos;
    Vector3f G_RefPrePos;
    Vector3f G_RefPos;
    Vector3f G_RefTargetPos;
    Vector3f G_RefVel;
}STANDTURN;

typedef struct OSQP {
    bool Flag;
    BASE base;
    COM com;

    float Mass;
    MatrixXf I_g                        = MatrixXf::Identity(3, 3);
    MatrixXf A                          = MatrixXf::Zero(6, 12);
    MatrixXf POS_CoM_CrossProduct       = MatrixXf::Zero(3, 12);
    Vector3f Gravity;
    VectorXf b                          = VectorXf::Zero(6);
    MatrixXf C                          = MatrixXf::Identity(12, 12);
    VectorXf u                          = VectorXf::Zero(12);
    VectorXf l                          = VectorXf::Zero(12);

    MatrixXf P                          = MatrixXf::Zero(12, 12);
    VectorXf q                          = VectorXf::Zero(12);

    // weight
    MatrixXf S                          = MatrixXf::Identity(6, 6);
    MatrixXf W                          = MatrixXf::Identity(12, 12);
    float alpha                         = 0.1; 

    // Selection Matrix
    MatrixXf Select                     = MatrixXf::Zero(12, 18);
    MatrixXf Jacobian                   = MatrixXf::Zero(12, 18);
        
    Vector3f boundary;
    Vector3f lower_boundary_local;
    Vector3f upper_boundary_local;
    
    c_int       exitflag = 0;
    c_float     data_P_x[78];
    c_int       data_P_nnz = 78;
    c_int       data_P_i[78 ];
    c_int       data_P_p[13];
    c_float     data_q[12];

    c_float     data_A_x[36];
    c_int       data_A_nnz = 12;

    c_int       data_A_i[36];
    c_int       data_A_p[13];
    
    c_int       data_m;
    c_int       data_n;
    
    c_float     data_l[20];
    c_float     data_u[20];

    //** WsJi: MPC variables (Note: variables should be float, not double)
    MatrixXf A_c = MatrixXf::Zero(13, 13);
    MatrixXf B_c = MatrixXf::Zero(13*horizontal_length, 12);
    MatrixXf A_hat = MatrixXf::Zero(13, 13);    // A_hat = A_c*dt + eye(13, 13);
    MatrixXf B_hat = MatrixXf::Zero(13*horizontal_length, 12);    // B_hat = B_c*dt;
    MatrixXf A_qp = MatrixXf::Zero(13*horizontal_length, 13);   // A_qp = [A_hat; A_hat^2; A_hat^3 ...]
    MatrixXf B_qp = MatrixXf::Zero(13*horizontal_length, 12*horizontal_length); // B_qp = 
    MatrixXf B_qp_transpose = MatrixXf::Zero(12*horizontal_length, 13*horizontal_length);

    int OSQPCalTime = 0;
    
    // WsJi:: OSQP
    // MatrixXd POS_CoM_2_RL_CrossProduct = MatrixXf::Zero(3, 3);
    // MatrixXd POS_CoM_2_RR_CrossProduct = MatrixXf::Zero(3, 3);
    // MatrixXd POS_CoM_2_FL_CrossProduct = MatrixXf::Zero(3, 3);
    // MatrixXd POS_CoM_2_FR_CrossProduct = MatrixXf::Zero(3, 3);
    VectorXf r_fl_vector = VectorXf::Zero(3*horizontal_length);
    VectorXf r_fr_vector = VectorXf::Zero(3*horizontal_length);
    VectorXf r_rl_vector = VectorXf::Zero(3*horizontal_length);
    VectorXf r_rr_vector = VectorXf::Zero(3*horizontal_length);

    MatrixXf w_I_g = MatrixXf::Zero(3, 3);
    MatrixXf w_I_g_transpose = MatrixXf::Zero(3, 3);
    MatrixXf w_I_g_inverse = MatrixXf::Zero(3, 3);


    // A_c, B_c 등은 MPC_Controller 내부에서 생성해주었다(osqp 가 아닌 다른 솔버를 쓸 지도 모르니깐?)
    // c_int       data_P_nnz_mpc = 78;
    // c_int       data_P_i_mpc[78];
    // c_int       data_P_p_mpc[13];
    // c_float     data_q_mpc[12];

    // c_float     data_A_x_mpc[36];
    // c_int       data_A_nnz_mpc = 12;

    // c_int       data_A_i_mpc[36];
    // c_int       data_A_p_mpc[13];
    
    // c_int       data_m_mpc;
    // c_int       data_n_mpc;
    
    // c_float     data_l_mpc[20];
    // c_float     data_u_mpc[20];

    // Matrix for OSQP
    MatrixXf P_mpc = MatrixXf::Zero(3 * 4 * horizontal_length, 3 * 4 * horizontal_length);
    MatrixXf qp_compute1_mpc = MatrixXf::Zero(3 * 4 * horizontal_length, 13 * horizontal_length);
    //MatrixXd 
    VectorXf q_mpc = VectorXf::Zero(horizontal_length * 12);

    // X References (= y)
    VectorXf X_ref_horizon = VectorXf::Zero(13 * horizontal_length);
    VectorXf X_mpc = VectorXf::Zero(13);


    // Weights
    MatrixXf L_mpc = MatrixXf::Zero(13 * horizontal_length, 13 * horizontal_length);
    MatrixXf K_mpc = MatrixXf::Identity(3 * 4 * horizontal_length, 3 * 4 * horizontal_length);

    VectorXf L_mpc_vec = VectorXf::Zero(13 * horizontal_length);
    VectorXf K_mpc_vec = VectorXf::Zero(3*4*horizontal_length);

    MatrixXf L_ori = MatrixXf::Zero(3, 3);
    MatrixXf L_pos = MatrixXf::Zero(3, 3);
    MatrixXf L_d_ori = MatrixXf::Zero(3, 3);
    MatrixXf L_d_pos = MatrixXf::Zero(3, 3);
    float weight_alpha = 1.;

    c_float data_l_mpc[horizontal_length * 20];
    c_float data_u_mpc[horizontal_length * 20];

    // OSQP for MPC
    //c_int       exitflag = 0;
    MatrixXf B_qp_transpose_time_L_mpc = MatrixXf::Zero(12*horizontal_length, 13*horizontal_length);
    MatrixXf Bt_L_B = MatrixXf::Zero(12*horizontal_length, 12*horizontal_length);


    MatrixXf C_ = MatrixXf::Zero(20, 12);

    MatrixXf C_mpc = MatrixXf::Zero(20 * horizontal_length, 12 * horizontal_length);
    c_int P_nnz_mpc = (144 * horizontal_length*horizontal_length + 12 * horizontal_length) / 2; // 1+2+...+(12*3) = (1+36)*(36/2) = 666
    c_float P_x_mpc[(144 * horizontal_length*horizontal_length + 12 * horizontal_length) / 2];
    c_int P_i_mpc[(144 * horizontal_length*horizontal_length + 12 * horizontal_length) / 2];
    c_int P_p_mpc[12 * horizontal_length + 1];
    c_float q_mpc_[horizontal_length * 12]; //
    c_int A_nnz_mpc = 36 * horizontal_length;
    c_float A_x_mpc[36 * horizontal_length];
    c_int A_i_mpc[36 * horizontal_length];
    c_int A_p_mpc[horizontal_length * 12 + 1];

    


    //** end of MPC variables

    
    // static friction coefficient
    float mu = 0.8;

    
    Vector3f tangential_direction1_local;
    Vector3f tangential_direction2_local;
    Vector3f SurfaceNormal_local;
    
    Vector3f tangential_direction1;
    Vector3f tangential_direction2;
    Vector3f SurfaceNormal;
    
    VectorXf Ref_GRF                        = VectorXf::Zero(12);
            
    VectorXf tmp_torque                     = VectorXf::Zero(12);
    VectorXd torque                         = VectorXd::Zero(18);
            
    // MatrixXf C_                             = MatrixXf::Zero(20, 12);
        
} OSQP;

typedef struct FORCE_ESTIMATION {
    VectorNd p                              = VectorNd::Zero(18);
    MatrixXd L_Gain                         = MatrixXd::Identity(18, 18);    
} FE;


typedef struct SLOPE {
    ENDPOINT RL_Foot, RR_Foot, FL_Foot, FR_Foot;
    BASE base;
    MatrixXf H                              = MatrixXf::Ones(4, 3);
    
    float** H_Matrix                        = matrix(1, 4, 1, 3);
    float** H_Matrix_transpose              = matrix(1, 3, 1, 4);
    float* LeastSquareSolution_Matrix       = fvector(1, 3);
    
    Vector4f FootState_X;
    Vector4f FootState_Y;
    Vector4f FootState_Z;

    Vector3f EstimatedAngle;
    Vector3f TmpEstimatedAngle;
    Vector3f LeastSquareSolution;       // plane coefficient [a b d]
    
    Vector3f TempLeastSquareSolution;   // plane coefficient [a b d]
    Vector3f normal;
    Vector3f V_vector;
    bool Flag;
    
} SLOPE;

typedef struct PRINT {
    VectorXf Data;
    int DataLength;
    int SaveTime;

    bool Trigger;
    
} PRINT;

typedef struct IMU {
    Vector3f DeltaOri_XYZ;
    Vector3f DeltaOri_ZYX;
    Vector3f DeltaAngularVel;

    Vector3f PreOri_XYZ;
    Vector3f PreOri_ZYX;
    Vector3f PreAngularVel;
    
    Vector3f Ori;
    Vector3f Ori_EulerZYX;
    Vector3f AngularVel;
    
    Vector3f Quat;
    Vector3f LinearAcc;

    Vector3f TempOri_ZYX;
    Vector3f PreTempOri_ZYX;
    Vector3f TempOri_XYZ;
    Vector3f PreTempOri_XYZ;

    Matrix3f RotMat;
    Vector3f Offset;

} IMU;

typedef struct Contact {
    Vector4f Foot;
    int Number;
} CONTACT;

typedef struct SOEM {
    VectorXf StatusWord     = VectorXf::Zero(JOINT_NUM);
    VectorXf RefCurrent     = VectorXf::Zero(JOINT_NUM);
    VectorXf RefTorque      = VectorXf::Zero(JOINT_NUM);
    
    float QPThreadTime;
    float IMUThreadTime;
    float MotionThreadTime;
    float TrajThreadTime;
} SOEM;

typedef struct HARDWARE {   
    VectorXf GearRatio                  = VectorXf::Zero(JOINT_NUM);
    VectorXf TorqueSensitivity          = VectorXf::Zero(JOINT_NUM);
    VectorXf RatedCurrent               = VectorXf::Zero(JOINT_NUM);
    VectorXf ABS_EncoderResolution      = VectorXf::Zero(JOINT_NUM);
        
    //* Friction
    VectorXf ViscousFriction            = VectorXf::Zero(JOINT_NUM);
    VectorXf CoulombFriction            = VectorXf::Zero(JOINT_NUM);
} HARDWARE;

typedef struct Friction {   
    VectorXf Current                    = VectorXf::Zero(JOINT_NUM);
    VectorXf Viscous                    = VectorXf::Zero(JOINT_NUM);
    VectorXf Coulomb                    = VectorXf::Zero(JOINT_NUM);
    VectorXf Inertia                    = VectorXf::Zero(JOINT_NUM);
    
    bool Activeflag;
} FRICTION;

typedef struct BaseFrame {
    COM com;
    BASE base;
} BASEFRAME;

typedef struct GlobalFrame {
    COM com;
    BASE base;
} GLOBALFRAME;

typedef struct PlaneFrame {
    COM com;
    BASE base;
    ENDPOINT RL_Foot, RR_Foot, FL_Foot, FR_Foot;
} PLANEFRAME;

typedef struct LowerbodyKinematic {
    //*PongBot Lower Body Kinematics
    Vector3f BASE_TO_HR_LENGTH;
    Vector3f HR_TO_HP_LENGTH;
    Vector3f HP_TO_KN_LENGTH;
    Vector3f KN_TO_EP_LENGTH;

    Vector3f RL_BASE_TO_HR;
    Vector3f RR_BASE_TO_HR;
    Vector3f FL_BASE_TO_HR;
    Vector3f FR_BASE_TO_HR;

    Vector3f RL_HR_TO_HP;
    Vector3f RR_HR_TO_HP;
    Vector3f FL_HR_TO_HP;
    Vector3f FR_HR_TO_HP;

    Vector3f RL_HP_TO_KN;
    Vector3f RR_HP_TO_KN;
    Vector3f FL_HP_TO_KN;
    Vector3f FR_HP_TO_KN;

    Vector3f RL_KN_TO_EP;
    Vector3f RR_KN_TO_EP;
    Vector3f FL_KN_TO_EP;
    Vector3f FR_KN_TO_EP;

    Vector3f RL_BASE_TO_EP;
    Vector3f RR_BASE_TO_EP;
    Vector3f FL_BASE_TO_EP;
    Vector3f FR_BASE_TO_EP;
    
} LBK;

typedef struct Torque {
    Vector18f joint;
    Vector18f task;
    bool CTC_Flag;

} TORQUE;

typedef struct Gain {
    Vector12f JointKp           = Vector12f::Zero(12);
    Vector12f JointKd           = Vector12f::Zero(12);
    Vector12f InitJointKp       = Vector12f::Zero(12);
    Vector12f InitJointKd       = Vector12f::Zero(12);
    Vector12f TargetJointKp     = Vector12f::Zero(12);
    Vector12f TargetJointKd     = Vector12f::Zero(12);
    
    Vector6f BasePosKp          = Vector6f::Zero(6);
    Vector6f BasePosKd          = Vector6f::Zero(6);
    Vector6f InitBasePosKp      = Vector6f::Zero(6);
    Vector6f InitBasePosKd      = Vector6f::Zero(6);
    Vector6f TargetBasePosKp    = Vector6f::Zero(6);
    Vector6f TargetBasePosKd    = Vector6f::Zero(6);
    
    Vector12f TaskKp            = Vector12f::Zero(12);
    Vector12f TaskKd            = Vector12f::Zero(12);
    Vector12f InitTaskKp        = Vector12f::Zero(12);
    Vector12f InitTaskKd        = Vector12f::Zero(12);
    Vector12f TargetTaskKp      = Vector12f::Zero(12);
    Vector12f TargetTaskKd      = Vector12f::Zero(12);

    Matrix3f CoM_QP_Kp;     // 
    Matrix3f CoM_QP_Kd;
    Matrix3f BASE_QP_Kp;    // 
    Matrix3f BASE_QP_Kd;
    
    Matrix3f InitCoM_QP_Kp;
    Matrix3f InitCoM_QP_Kd;
    Matrix3f InitBASE_QP_Kp;
    Matrix3f InitBASE_QP_Kd;

    float Torque;
} GAIN;

typedef struct Walk_Parameter_new{

    

    // 시간에 대한 것
    float t_p = 0.3;
    float t_swing = 0.15;
    float t_stance = 0.15;

    float time_gait = 0.;
    float time_gait_prev = 0.;
    float dt_gait = 0.002;

    // CoM 에 대한 것
    Vector3f p_CoM_desired = VectorXf::Zero(3);
    Vector3f p_CoM_now = VectorXf::Zero(3);
    Vector3f p_CoM_prev = VectorXf::Zero(3);

    Vector3f p_CoM_real_now = VectorXf::Zero(3);    // 다리 고려한 리얼 위치
    Vector3f p_CoM_real_desired = VectorXf::Zero(3);    // 다리 고려한 리얼 위치

    Vector3f v_CoM_desired = VectorXf::Zero(3);
    Vector3f v_CoM_prev = VectorXf::Zero(3);

    Vector6f CoM_traj_x_5thpoly_coeffs = VectorXf::Zero(6);
    Vector6f CoM_traj_y_5thpoly_coeffs = VectorXf::Zero(6);
    Vector6f ori_CoM_traj_psi_5thpoly_coeffs = VectorXf::Zero(6);

    Vector3f p_CoM_traj = VectorXf::Zero(3);
    



    // 방위자세에 대한 것
    Vector3f ori_CoM_desired = VectorXf::Zero(3);
    Vector3f d_ori_CoM_desired = VectorXf::Zero(3);
    Vector3f ori_CoM_now = VectorXf::Zero(3);
    Vector3f d_ori_CoM_now = VectorXf::Zero(3);

    Vector3f ori_CoM_traj = VectorXf::Zero(3);
    Vector3f d_ori_CoM_traj = VectorXf::Zero(3);


    // 다리에 대한 것 
    // FL
    Vector3f p_com2FL_offset = VectorXf::Zero(3);
    Vector3f p_FL_desired = VectorXf::Zero(3);
    Vector3f p_FL_now = VectorXf::Zero(3);
    Vector3f p_FL_traj = VectorXf::Zero(3);
    Vector3f p_FL_traj_bodyframe = VectorXf::Zero(3);
    bool FL_contact = true;

    // FR
    Vector3f p_com2FR_offset = VectorXf::Zero(3);
    Vector3f p_FR_desired = VectorXf::Zero(3);
    Vector3f p_FR_now = VectorXf::Zero(3);
    Vector3f p_FR_traj = VectorXf::Zero(3);
    Vector3f p_FR_traj_bodyframe = VectorXf::Zero(3);
    bool FR_contact = true;
    
    // RL
    Vector3f p_com2RL_offset = VectorXf::Zero(3);
    Vector3f p_RL_desired = VectorXf::Zero(3);
    Vector3f p_RL_now = VectorXf::Zero(3);
    Vector3f p_RL_traj = VectorXf::Zero(3);
    Vector3f p_RL_traj_bodyframe = VectorXf::Zero(3);
    bool RL_contact = true;    
    
    // RR
    Vector3f p_com2RR_offset = VectorXf::Zero(3);
    Vector3f p_RR_desired = VectorXf::Zero(3);
    Vector3f p_RR_now = VectorXf::Zero(3);
    Vector3f p_RR_traj = VectorXf::Zero(3);
    Vector3f p_RR_traj_bodyframe = VectorXf::Zero(3);
    bool RR_contact = true;


    // 착지점
    Vector3f FL_footstep_pos = VectorXf::Zero(3);
    bool FL_footstep_pos_flag = false;
    

    Vector3f FR_footstep_pos = VectorXf::Zero(3);
    bool FR_footstep_pos_flag = false;

    Vector3f RL_footstep_pos = VectorXf::Zero(3);
    bool RL_footstep_pos_flag = false;

    Vector3f RR_footstep_pos = VectorXf::Zero(3);
    bool RR_footstep_pos_flag = false;



    // 조이스틱
    Vector3f v_CoM_joystick = VectorXf::Zero(3);
    

    // 기타 정보
    unsigned int gait_type = 1; // 0: WALK, 1: TROT
    unsigned int gait_type_target = 1;
    bool gait_type_changed = false;
    
    unsigned int gait_phase = STANCE_PHASE;

    unsigned int swingLeg = LEGPHASE_RL;
    unsigned int swingLeg_prev = LEGPHASE_RL;


    bool gait_on_off_target = true;
    bool gait_on_off_now = true;
    
    float foot_traj_height = 0.;
    // float foot_traj_height = 0.33;
    
    // Stance & Swing On/Off flag
    bool Stance_On_flag = false;
    bool Swing_On_flag = false;

    Walk_Parameter_new()    
    {
        // 일부 벡터들의 초기화
        gait_type = 1;
        gait_type_target = 1;


        p_CoM_desired << 0., 0., 0.4;
        p_CoM_real_desired = p_CoM_desired;
        // p_CoM_desired << 0., 0., 0.5;

        // 실제 퐁봇
        // p_com2FL_offset <<  0.32,  0.218, -0.4;
        // p_com2FR_offset <<  0.32, -0.218, -0.4;
        // p_com2RL_offset << -0.32,  0.218, -0.4;
        // p_com2RR_offset << -0.32, -0.218, -0.4;

        // 현재 이 시뮬레이션의 퐁봇 치수
        p_com2FL_offset <<  0.29804,  0.1988, -0.4;
        p_com2FR_offset <<  0.29804, -0.1988, -0.4;
        p_com2RL_offset << -0.29804,  0.1988, -0.4;
        p_com2RR_offset << -0.29804, -0.1988, -0.4;

        // p_com2FL_offset <<  0.29804,  0.1988, -0.5;
        // p_com2FR_offset <<  0.29804, -0.1988, -0.5;
        // p_com2RL_offset << -0.29804,  0.1988, -0.5;
        // p_com2RR_offset << -0.29804, -0.1988, -0.5;
        
        p_FL_desired = p_CoM_desired + p_com2FL_offset;
        p_FR_desired = p_CoM_desired + p_com2FR_offset;
        p_RL_desired = p_CoM_desired + p_com2RL_offset;
        p_RR_desired = p_CoM_desired + p_com2RR_offset;


        // 다리를 드는 디폴트 높이
        // foot_traj_height = 0.25;    // 0.15m
        // foot_traj_height = 0.33;    // 0.15m
        foot_traj_height = 0.15;    // 0.15m
    }


} WALK_PARAMETER_NEW;

// WALK_PARAMETER_NEW::Walk_Parameter_new()
// {
//     p_com2FL_offset << 0, 1, 2;
// }


typedef struct Walk_Parameter{
    
    bool WalkStart = false;

    // flag for walking sequence: two flags can not be true in same time.
    // 1: FR, 2: RL, 3: FL, 4: RR. 0: none
    unsigned int walking_leg_choose = 0;
    
    
    // simulation and walking parameter
    float sim_dt = 0.001;
    float walk_time;
    float support_time;

    float phase_time;
    float phase_length;
    float walk_length;
    float support_length;

    size_t phase_index = 0;
    float pdt = 0.;        // phase_dt
    float walkingdt = 0.;

    // input velocity and position from Joystick
    float vel_joy = 0.;
    float vel_joy_prev = 0.;
    float vel_COM = 0.;
    float vel_COM_prev = 0.;

    float vel_temp = 0.3;

    float pos_fin = 0.;    // final (local)position for each phase. vel_COM * phase_time

    float swing_height = 0.;   // didn't input from the Joystick, but...



    // CoM 5th polynomial coefficients
    VectorXf coefs = VectorXf::Zero(6);

    
//     Vector3f RL_pos = VectorXf::Zero(3);
//     Vector3f RL_swing_pos = VectorXf::Zero(3);
//     Vector3f RL_swing_prev = VectorXf::Zero(3);
//     Vector3f RL_support_pos = VectorXf::Zero(3);    // = -CoM_pos
//     Vector3f RL_walk_temp = VectorXf::Zero(3);
//     Vector3f RL_offset = VectorXf::Zero(3);
//     Vector3f RL_pos_with_offset = VectorXf::Zero(3);


    // position vector of the COM and end effectors
    VectorXf CoM_pos = Vector3f::Zero(3);
    VectorXf CoM_global_pos = Vector3f::Zero(3);
    VectorXf CoM_global_pos_prev = Vector3f::Zero(3);
    //VectorXd CoM_global_pos = Vector3d::Zero(3);

    VectorXf RL_pos = Vector3f::Zero(3);
    VectorXf RL_swing_pos = Vector3f::Zero(3);
    VectorXf RL_swing_pos_prev = Vector3f::Zero(3);
    VectorXf RL_support_pos = Vector3f::Zero(3);
    VectorXf RL_walk_temp = Vector3f::Zero(3);
    VectorXf RL_offset = Vector3f::Zero(3);
    VectorXf RL_pos_with_offset = VectorXf::Zero(3);

    VectorXf RR_pos = Vector3f::Zero(3);
    VectorXf RR_swing_pos = Vector3f::Zero(3);
    VectorXf RR_swing_pos_prev = Vector3f::Zero(3);
    VectorXf RR_support_pos = Vector3f::Zero(3);
    VectorXf RR_walk_temp = Vector3f::Zero(3);
    VectorXf RR_offset = Vector3f::Zero(3);
    VectorXf RR_pos_with_offset = VectorXf::Zero(3);
    
    VectorXf FL_pos = Vector3f::Zero(3);
    VectorXf FL_swing_pos = Vector3f::Zero(3);
    VectorXf FL_swing_pos_prev = Vector3f::Zero(3);
    VectorXf FL_support_pos = Vector3f::Zero(3);
    VectorXf FL_walk_temp = Vector3f::Zero(3);
    VectorXf FL_offset = Vector3f::Zero(3);
    VectorXf FL_pos_with_offset = VectorXf::Zero(3);

    VectorXf FR_pos = Vector3f::Zero(3);
    VectorXf FR_swing_pos = Vector3f::Zero(3);
    VectorXf FR_swing_pos_prev = Vector3f::Zero(3);
    VectorXf FR_support_pos = Vector3f::Zero(3);
    VectorXf FR_walk_temp = Vector3f::Zero(3);
    VectorXf FR_offset = Vector3f::Zero(3);
    VectorXf FR_pos_with_offset = VectorXf::Zero(3);

    
    // VectorXf RL_pos_with_offset = Vector3f::Zero(3);
    // VectorXf FR_pos_with_offset = Vector3f::Zero(3);
    // VectorXf RR_pos_with_offset = Vector3f::Zero(3);
    // VectorXf FL_pos_with_offset = Vector3f::Zero(3);
    


} WALK_PARAMETER;

class CRobot {
public:

    // 재배치 - 안쓰는 것으로 보이거나 내 관심사가 아닌 것들
    BASEFRAME BaseFrame;    // 안쓰는 것으로 보임
    GLOBALFRAME GlobalFrame;    // 안쓰는 것으로 보임
    PLANEFRAME PlaneFrame;  // 안쓰는 것으로 보임
    JOINT* joint;   // 안쓰는 것으로 보임
    WHEEL wheel; // 내 관심사항은 아님
    FE ForceEstimation; // 안쓰는 것으로 보임
    SOEM soem; // 안쓰는 것으로 보임 (프린트에만 살아있음)
    CONTACT contact;    // 안쓰는 것으로 보임



    COM com;
    BASE base;
    
    ENDPOINT RL_Foot, RR_Foot, FL_Foot, FR_Foot; 
    TORQUE torque;
    
    GAIN gain; 
    WALK walking;
    STANDTURN standturn;
    LBK lowerbody;
    OSQP osqp;
    
    SLOPE slope; 
    PRINT print;
    IMU imu;
    HARDWARE hardware;
    FRICTION friction;
    

    // WsJi
    // WALKINGPATTERN mWalkingPattern;
    // 이 아래 함수들도 싹 뒤엎긴 해야한다
    WALK_PARAMETER walk_para;
    void PatternGenerator_Walk(float walk_time, float support_time);   // generate patterns for CoM and four legs.
    void PatternGenerator_Walk_Init(float walk_time, float support_time);  // Initialize parameters for Trot
    Vector6f coefficient_5thPolyNomial_WsJi(Vector3f init_value, Vector3f final_value, float tp);

    
    // 과제에서 만든 함수들(Matlab) 을 C++ 화 시켜야 하는데.....
    // Matlab 에서 만들 땐 출력을 예를 들어서, [gait_type, gait_type_changed] = gait_selector(..., ..., ...) 형태로 만들 수가 있었다.
    // 하지만 C++ 에서는 그런 형태의 생성이 어렵기 때문에... 어쩔 수 없이 void 형태로 만들어야하나...??????????????????
    // 그렇다면 모든 함수들을 void 형태로 만드는 게 나을까?
    // 매개변수에 입, 출력이 모두 들어가도록 만드는 게 나을 수 있겠다.



    // WALK_PARAMETER_NEW walk_para_array[horizontal_N];
    WALK_PARAMETER_NEW walk_para_array;

    // WALK_PARAMETER_NEW walk_para_new;

    void LocomotionPatternGenerator(WALK_PARAMETER_NEW& Variables);

    void Optimization_Process(const unsigned int gait_type, VectorXf vel_com_target, float& T_period, float& T_stance, float& T_swing, Vector3f* vel_com_desired, Vector3f* d_ori_com_desired);
    // unsigned int SwingLegChecker(const unsigned int swingLeg_prev, const unsigned int gait_type);
    //void SwingLegChecker(const unsigned int swingLeg_prev, const unsigned int gait_type, unsigned int& swingLeg);
    void SwingLegChecker(const unsigned int gait_type, unsigned int& swingLeg);

    void Timer_for_gait_traj(const float time_gait_prev, const float dt_gait, const float T_period, float& time_gait);    
    void Timer_for_gait_traj(const float dt_gait, const float T_period, float& time_gait);    
    void Timer_for_gait_traj(const float dt_gait, const float T_period, float& time_gait, bool& stance_flag, bool& swing_flag); 
    
    void Test_vectorxf_reference(Vector3f* temp);

    void Gait_Selector(unsigned int gait_type_target, unsigned int& gait_type, bool& gait_type_changed);

    // 이 함수같은 경우는 입출력이 너무 많아서 걱정인데...
    void PrevDataSave(Vector3f* p_CoM_now, Vector3f p_CoM_desired, Vector3f p_FL_desired, Vector3f p_FR_desired, Vector3f p_RL_desired, Vector3f p_RR_desired, Vector3f v_CoM_desired,
                      Vector3f* p_CoM_prev, Vector3f* p_FL_now, Vector3f* p_FR_now, Vector3f* p_RL_now, Vector3f* p_RR_now, Vector3f* v_CoM_prev);

    void GaitSelector(const unsigned int gait_type_target, const unsigned int gait_type_now, const float slope_angle, const float stair_height, unsigned int& gait_type, bool& gait_type_changed);

    float coswave_modified(const float pos_start, const float pos_end, const float time, const float T_p);
    void poly_5th_coeffs(float x_init, float dx_init, float d2x_init, float x_desired, float dx_desired, float d2x_desired, float tp, Vector6f* Coeff_State);
    float bezier_curve_modified(float pos_start, float pos_end, float time, float t_p, unsigned int traj_axis);

    Vector3f CoM_EndPoint_Generator(const float t_p, const float t_stance, const float t_swing, const Vector3f com_speed_desired, const unsigned int gait_type, const unsigned int gait_phase,
                                    const bool gait_type_changed, const unsigned int swing_leg, const Vector3f CoM_pos_now, const Vector3f CoM_pos_prev,
                                    const Vector3f FL_pos, const Vector3f FR_pos, const Vector3f RL_pos, const Vector3f RR_pos);

    void CoM_EndPoint_Generator_modified(const float t_p, const float t_stance, const float t_swing, const Vector3f com_speed_desired, const unsigned int gait_type, const unsigned int gait_phase,
                                            const bool gait_type_changed, const unsigned int swing_leg, const Vector3f CoM_pos_now, const Vector3f CoM_pos_prev,
                                            const Vector3f FL_pos, const Vector3f FR_pos, const Vector3f RL_pos, const Vector3f RR_pos,
                                            const Vector3f ori_CoM_now, const Vector3f v_CoM_desired, const Vector3f d_ori_CoM_desired,
                                            Vector3f* CoM_pos_desired, Vector3f* ori_CoM_desired);

                                            

    Vector3f CoM_Traj_Generator(const bool gait_on_off_now, const unsigned int gait_type, const unsigned int gait_phase,
                                const Vector3f p_CoM_now, const Vector3f p_CoM_desired, const Vector3f v_CoM_prev, const Vector3f v_CoM_desired,
                                const float t_stance, const float t_swing, const float time_gait, const float dt_gait);

    Vector3f CoM_Traj_Generator(const bool gait_on_off_now, const unsigned int gait_type, const unsigned int gait_phase,
                                const Vector3f p_CoM_now, const Vector3f p_CoM_desired, const Vector3f v_CoM_prev, const Vector3f v_CoM_desired,
                                const Vector3f ori_CoM_now, const Vector3f ori_CoM_desired,
                                const float t_stance, const float t_swing, const float time_gait, const float dt_gait,
                                const Vector6f CoM_traj_x_5thpoly_coeffs, const Vector6f CoM_traj_y_5thpoly_coeffs, const Vector6f ori_CoM_traj_psi_5thpoly_coeffs);
                                //,                                Vector6f CoM_traj_x_5thpoly_coeffs, Vector6f CoM_traj_y_5thpoly_coeffs);


    void CoM_Traj_Generator_modified(const bool gait_on_off_now, const unsigned int gait_type, const unsigned int gait_phase,
                                    const Vector3f p_CoM_now, const Vector3f p_CoM_desired, const Vector3f v_CoM_prev, const Vector3f v_CoM_desired,
                                    const Vector3f ori_CoM_now, const Vector3f ori_CoM_desired,
                                    const float t_stance, const float t_swing, const float time_gait, const float dt_gait,
                                    const Vector6f CoM_traj_x_5thpoly_coeffs, const Vector6f CoM_traj_y_5thpoly_coeffs, const Vector6f ori_CoM_traj_psi_5thpoly_coeffs,
                                    Vector3f* p_CoM_traj, Vector3f* ori_CoM_traj);

    Vector3f Foot_EndPoint_Generator(const float t_p, const float t_stance, const Vector3f v_CoM_desired, const unsigned int gait_type, const unsigned int gait_phase,
                                     const Vector3f Foot_pos_now, const Vector3f CoM_pos_desired, const Vector3f Foot_pos_offset, float yaw_desired);

    Vector3f Foot_EndPoint_Generator(const float t_p, const float t_stance, const Vector3f v_CoM_desired, const unsigned int gait_type, const unsigned int gait_phase,
                                     const Vector3f Foot_pos_now, const Vector3f CoM_pos_desired, const Vector3f Foot_pos_offset, float global_z_height, bool user_input,
                                     float yaw_desired
                                     );

    void Foot_Contact_Checker(const unsigned int swing_leg, const unsigned int gait_phase,
                              bool& FL_contact, bool& FR_contact, bool& RL_contact, bool& RR_contact);

    void Foot_Traj_Generator(const bool gait_on_off_now, const unsigned int gait_type, const unsigned int gait_phase, const unsigned int swing_leg,
                                                 const Vector3f p_RL_now, const Vector3f p_RR_now, const Vector3f p_FL_now, const Vector3f p_FR_now,
                                                 const Vector3f p_RL_desired, const Vector3f p_RR_desired, const Vector3f p_FL_desired, const Vector3f p_FR_desired,
                                                 const float t_stance, const float t_swing, const float time_gait, const float foot_traj_height,
                                                 Vector3f* p_FL_traj_, Vector3f* p_FR_traj_, Vector3f* p_RL_traj_, Vector3f* p_RR_traj_);

    // MPC
    VectorXf MPC_controller(unsigned int horizon_length, float samplingtime, unsigned int total_legs);
    MatrixXf generateFootTraj_Trot_modified(float com_time, float motion_time, int Startfoot, int Startfoot_prev, int k);
    VectorXf generateCoMTraj_modified(float motion_time, int k);
    void MPC_Gain_Tuning(void);

    OSQP osqp_mpc;
    // VectorXf initializeOSQP_MPC(OSQP osqp_mpc);
    // VectorXf ProcessOSQP_MPC(OSQP osqp_mpc);

    // 다족형 과제 컨셉에 맞춘 기능들 모듈화 작업 진행(230620~)
    void OptimizationProcess(const VectorXf& com_target,\
                            float& stance_time, float& swing_time,\
                            float& Xstep, float& Ystep, float& Turnstep);
    
    void ChooseWalkOrTrot(const float& stance_time, const float& swing_time, const float& Xstep, const float& Ystep, const float& Turnstep,\
                          int& walktype);    // 추후 만들어야 하는 내용

    void StateTransition(const int& walktype,\
                         float& totalContactLegs);

    void CoMTrajectory(const int& walktype, const VectorXf& com_target_modified,\
                        Vector3f& com_ref);

    


    int mWalkTypeSelection = 1;

    enum WalkType{
        TROTMODE = 1, WALKMODE
    };

    void generateCoMTraj_new(float motion_time);

    void setWalkingStep();  // 기존에 있는 함수의 오버로딩
    void calculateCoM5thPolynomialCoeff();


    void OSQP_Init(void);

    // 임시 변수 (특정 함수들 안에서 static 으로 선언된 변수들을 관측하기 위한)
    float temp_com_motion_time = 0.;
    float temp_foot_motion_time = 0.;
    VectorXf temp_com_ref = VectorXf::Zero(3*200); //
    VectorXf temp_com_vel_ref = VectorXf::Zero(3*200);
    
    MatrixXf temp_legs_ref = MatrixXf::Zero(3*200, 4);

    // VectorXf temp_com2FL_ref = VectorXf::Zero(3*200);
    // VectorXf temp_com2FR_ref = VectorXf::Zero(3*200);
    // VectorXf temp_com2RL_ref = VectorXf::Zero(3*200);
    // VectorXf temp_com2RR_ref = VectorXf::Zero(3*200);

    VectorXf temp_com2FL_ref = VectorXf::Zero(3*horizontal_N);
    VectorXf temp_com2FR_ref = VectorXf::Zero(3*horizontal_N);
    VectorXf temp_com2RL_ref = VectorXf::Zero(3*horizontal_N);
    VectorXf temp_com2RR_ref = VectorXf::Zero(3*horizontal_N);

    VectorXf temp_Foot_to_CoM_ref = VectorXf::Zero(3*horizontal_N);  // global 좌표계에서 com의 미래 레퍼런스 모음인 temp_com_ref 를 "다리 좌표계" 에서 관측하기 위한 값
    VectorXf temp_Foot_to_CoM_vel_ref = VectorXf::Zero(3*horizontal_N);

    VectorXf temp_Offset_Global_between_FootCoord = VectorXf::Zero(3);  // "매 시점" 에서 레퍼런스를 계산할 때, 글로벌 좌표계와 매 시점의 다리 좌표계 사이의 오프셋. referenceUpdate 에서 계산해주면 될 것 같다.

    VectorXf temp_FR_ref = VectorXf::Zero(30);    
    VectorXf temp_FR_ref2 = VectorXf::Zero(3);

    //MatrixXf temp_contact_foots = MatrixXf::Zero(4, 200);   // 네 개의 다리를 미래 200개까지 컨택트 예상 여부를 저장
    MatrixXf temp_contact_foots = MatrixXf::Zero(4, horizontal_N);   // 네 개의 다리를 미래 200개까지 컨택트 예상 여부를 저장

    float temptime = 0.;

    


    enum JointNumber {
        FLHR = 0, FLHP, FLKN, FRHR, FRHP, FRKN, RLHR,
        RLHP, RLKN, RRHR, RRHP, RRKN, FLWHL, FRWHL, RLWHL, RRWHL,
    };

    enum FootIndex {
        FL_FootIndex = 0, FR_FootIndex, RL_FootIndex, RR_FootIndex
    };
    
    enum {
        Fx = 0, Fy, Fz
    };

    CRobot();
    CRobot(const CRobot& orig);
    virtual ~CRobot();

    //Functions
    void initializeCRobot(Model* getModel);
    void initializeSystemID(void);
    void initializeParameter(void);
    void initializeOSQP(void);
    void initializeHardWare(void);
    
    void setRobotModel(Model* getModel);
    void StateUpdate(void);
    void ComputeTorqueControl(void);

    void GetFrictionCoefficient(float readytime);
    void SlopeEstimation(void);
    void WalkingReady(float readytime);
    void HomePose(float readytime);
    void setMode(bool ModeSelection, int TypeSelection);
    void setPrintData(int DataLength, int SaveTime);
    
    void RobotStateUpdate();
    void kinematicsUpdate();
    void referenceUpdate();
    
    void GenerateFriction(void);
    void ContactForceEstimation();
    void SaveData();
    void TorqueOff(void);

    Vector12f computeIK(Vector12f EndPoint);
    Vector3f GetCOM(Vector3f BasePos, Vector3f BaseOri, Vector12f JointAngle);
    
    
    void ProcessOSQP(void);
    void PARA_Init(void);

    float cosWave(float Amp, float Period, float Time, float InitPos);
    Vector3f cosWave(Vector3f Amp, float Period, float Time, Vector3f InitPos);
    float differential_cosWave(float Amp, float Period, float Time);
    Vector3f differential_cosWave(Vector3f Amp, float Period, float Time);
        
    MatrixXf X_Rot(float q, int size);
    MatrixXf Y_Rot(float q, int size);
    MatrixXf Z_Rot(float q, int size);
    Vector3f RotMat2EulerZYX(Matrix3f RotMat);
    Matrix3f EulerZYX2RotMat(Vector3f RotationAngle);
    
    Vector3f local2Global(Vector3f Vector, Matrix3f RotMat);
    Vector3f global2Local(Vector3f Vector, Matrix3f RotMat);
    Matrix3f SkewSymmetricMat(Vector3f AngularVelocity);
    
    void setRotationZYX(Vector3f ori, float** out_T);
    
    void MomentumObserver(double f_cutoff = 20);
    
    void Trot(void);
    void Trot_init(void);
    void Trot_stop(void);
    void Stand(void);
    void StairWalk_Init(void);
    void StairWalk(void);
    void IMUNulling(void);
    void IMU_Update(void);
    void setWalkingSpeedLimit(void);
    void setWalkingStep(float FBsize, float LRsize, float Turnsize, int Startfoot);
    void GainSchedule(int StartFoot, float MotionTime, float PeriodTime);
    void generateCoMTraj(float motion_time);
    void generateFootTraj_Trot(float motion_time, int Startfoot);
    void generateFootTraj_Stair(float motion_time, int Startfoot);

    



    void coefficient_5thPolyNomial(Vector3f pre_value, Vector3f final_value, float final_time, float *output);
    
    float Function_5thPolyNomial(Vector6f coeff, float time);
    float Function_5thPolyNomial_dot(Vector6f coeff, float time);
    float Function_5thPolyNomial_2dot(Vector6f coeff, float time);
    
    VectorXf Set_8thBezierControlPoint(Vector3f StartPoint, Vector3f EndPoint, int Axis, Matrix3f Rotation);
    float Function_8thBezierCurve(VectorXf ControlPoint, float Period, float Time);
    float Function_8thBezierCurve_dot(VectorXf ControlPoint, float Period, float Time);
    
    void setMappingE_ZYX(Vector3f ori, float** out_T);
    void invMappingE_ZYX(Vector3f ori, float** out_T);
    void set_skew_symmetic(Vector3f vec, float** out_T);
    void set_5thPolyNomialMatrix(float final_time, float** out_T);
    
    float** Ref_mapE_B          = matrix(1, 3, 1, 3);
    float** Ref_mapE_B_inv      = matrix(1, 3, 1, 3);
    
    float** Current_mapE_B      = matrix(1, 3, 1, 3);
    float** Current_mapE_B_inv  = matrix(1, 3, 1, 3);
    // ============= RBDL ============== //
    
    float tasktime;
    float onesecSize;
    
    Quaternion InitQuaternion;
    RigidBodyDynamics::Model* m_pModel; //* URDF Model
    MatrixNd M_term                 = MatrixNd::Zero(18, 18);
    MatrixNd M_term_hat             = MatrixNd::Zero(18, 18);
    VectorNd M_term_Torque        = VectorNd::Zero(18);
    VectorNd M_term_tmpTorque       = VectorNd::Zero(18);
    VectorNd hatNonLinearEffects    = VectorNd::Zero(18);
    
    VectorNd G_term                 = VectorNd::Zero(18);
    double G_term_Weight             = 0;
    
    VectorNd C_term                 = VectorNd::Zero(18);
    VectorNd JointRefAcc            = VectorNd::Zero(18);
    
    VectorNd JointCurrentVel        = VectorNd::Zero(18);
    
    VectorXd CTC_Torque             = VectorXd::Zero(18);
    VectorXd ActualTorque           = VectorXd::Zero(12);
    VectorXd StatusWord             = VectorXd::Zero(12);
    
    int Mode, Type;
    
    // ============= PARAMETER =============== //    
    float* B_Base2CoM_RefPos            = fvector(1, 3);
    float* B_CoM2Base_RefPos            = fvector(1, 3);

    float* G_Base2CoM_RefPos            = fvector(1, 3);
    float* G_CoM2Base_RefPos            = fvector(1, 3);
    
    
    MatrixXf ROT_WORLD2BASE             = MatrixXf::Identity(3, 3);
    MatrixXf ROT_WORLD2BASE_TOTAL       = MatrixXf::Identity(12, 12);

    MatrixXf ROT_BASE2WORLD             = MatrixXf::Identity(3, 3);
    MatrixXf ROT_BASE2WORLD_TOTAL       = MatrixXf::Identity(12, 12);

    int joint_DoF;                      //Joint Degrees of Freedom
    unsigned int ControlMode;
    unsigned int CommandFlag;

    // Rotation Matrix
    float** Ref_C_gb                    = matrix(1, 3, 1, 3);       //     Ref_R_g_2_b   is the rotation matrix which transforms from global to base coordinate system
    float** Ref_C_bg                    = matrix(1, 3, 1, 3);       //     Ref_R_b_2_g   is the rotation matrix which transforms from base to global coordinate system
    float** Current_C_gb                = matrix(1, 3, 1, 3);       // Current_R_g_2_b   is the rotation matrix which transforms from global to base coordinate system
    float** Current_C_bg                = matrix(1, 3, 1, 3);       // Current_R_b_2_g   is the rotation matrix which transforms from base to global coordinate system

    float** Ref_B_w_gb_skew             = matrix(1, 3, 1, 3);       // B_w_gb is the skew symmetric matrix of angular velocity vector which describes the rotational motion of Base frame with respect to Global frame in Base frame coordinate system.
    float** Ref_G_w_gb_skew             = matrix(1, 3, 1, 3);       // G_w_gb is the skew symmetric matrix of angular velocity vector which describes the rotational motion of Base frame with respect to Global frame in Global frame coordinate system.
    float** Current_B_w_gb_skew         = matrix(1, 3, 1, 3);       // B_w_gb is the skew symmetric matrix of angular velocity vector which describes the rotational motion of Base frame with respect to Global frame in Base frame coordinate system.
    float** Current_G_w_gb_skew         = matrix(1, 3, 1, 3);       // G_w_gb is the skew symmetric matrix of angular velocity vector which describes the rotational motion of Base frame with respect to Global frame in Global frame coordinate system.

    
    float** RefTarget_C_gb              = matrix(1, 3, 1, 3);       // RefTarget_R_g_2_b is the Target rotation matrix which transforms from global to base coordinate system
    
    // ===================== OSQP ====================== //
    // Workspace structures
    OSQPWorkspace   *OSQP_work;
    OSQPSettings    *OSQP_settings  = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    OSQPData        *OSQP_data      = (OSQPData *) c_malloc(sizeof (OSQPData));
    // ===================== END ====================== //
            
private:
    BodyKinematic BodyKine;
    
   /* Degree Of Freedom 
    * Foot : X Y Z 
    * Base : X Y Z Roll Pitch Yaw
    */
    int FootDOF   = 3;
    int BaseDOF   = 6;
    
    MatrixNd J_BASE;
    MatrixNd J_FL;
    MatrixNd J_FR;
    MatrixNd J_RL;
    MatrixNd J_RR;
    
    MatrixNd J_FL2;
    MatrixNd J_FR2;
    MatrixNd J_RL2;
    MatrixNd J_RR2;
    
    Matrix3f B_J_FL;
    Matrix3f B_J_FR;
    Matrix3f B_J_RL;
    Matrix3f B_J_RR;

    VectorNd RobotState;
    VectorNd RobotStatedot;
    VectorNd RobotState2dot;
    
    VectorNd RobotState_local;
    
    float** cur_mapE_B = matrix(1, 3, 1, 3);
    float** cur_inv_mapE_B = matrix(1, 3, 1, 3);
};

#endif /* CROBOT_H */

Eigen::MatrixXf rot_z(float angle);

Eigen::MatrixXf rot_y(float angle);

Eigen::MatrixXf rot_x(float angle);

Matrix3f skew_matrix(Vector3f vec);