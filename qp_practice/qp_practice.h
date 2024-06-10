/*
 * ============================================================
 * qp_practice Class Header
 * 
 * Robotics & Control Lab.
 * 
 * File                 : QuadraticProgram.h
 * Author               : BK Cho
 *
 * First developer      : Jeong Hwan. Jang
 * Second developer     :
 * 
 * Update date          : 2024. 06. 10 by Jeong Hwan. Jang
 * ============================================================
 */

#ifndef qp_practice_H
#define qp_practice_H

//* Header file for C++
#include <stdio.h>
#include <iostream> 
#include <boost/bind.hpp>

//* Header file for Gazebo and ROS
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>
// #include <ignition/math.hh>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>


#include <chrono>   // 정밀 시간 측정을 위해 넣었음

#include <Eigen/Dense>     
//* Eigen library
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Matrix5f;
using Eigen::Matrix6f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Vector5f;
using Eigen::Vector6f;

#include <rbdl/rbdl.h>                              // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h>   

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

// #include "CRobot/CRobot.h"
#include "osqp.h"

typedef struct OSQP {

    //* ========== OSQP ========== *//
    /*
     * OSQP 구조체 입니다.
     *
     * 사용될 변수, Matrix, Vector 등을 정의해 놓았습니다.
     * 
     * 논문 [High-slope Terraian Locomotion for Torque-Controlled Quadruped Robots] 에 나온 Matrix들에 대한 설명은 qp_practice.cpp 상단에 설명되어 있습니다.
     * 
     * 각 변수, Matrix, Vector 의 크기를 모두 2로 바꿔놓았으니 크기에 대한 고민 후 새로 정의 하시길 바랍니다.
     * 
     * ex) Ref_GRF 의 경우 4개의 다리에 x, y, z 방향 GRF가 포함되어야 하니 12 의 크기를 가지겠죠?
     *
     * 
     * 이 구조체 내에 정의되지 않은 변수들은 각자 정의하여 사용하시면 됩니다.
     * 
     * ex) CoM_RefAcc, G_Base_RefAngularVel 등..   
     * 
     * 
     */ 
    float Mass;

    MatrixXf I_g                                        = MatrixXf::Identity(2, 2);
    MatrixXf A                                          = MatrixXf::Zero(2, 2);
    MatrixXf POS_CoM_CrossProduct                       = MatrixXf::Zero(2, 2);
    Vector3f Gravity;
    VectorXf b                                          = VectorXf::Zero(2);
    MatrixXf C                                          = MatrixXf::Identity(2, 2);
    VectorXf u                                          = VectorXf::Zero(2);
    VectorXf l                                          = VectorXf::Zero(2);

    MatrixXf P                                          = MatrixXf::Zero(2, 2);
    VectorXf q                                          = VectorXf::Zero(2);

    // weight
    MatrixXf S                                          = MatrixXf::Identity(2, 2);
    MatrixXf W                                          = MatrixXf::Identity(2, 2);
    float alpha                                         = 0.1; 

    // Selection Matrix
    MatrixXf Select                                     = MatrixXf::Zero(2, 2);
    MatrixXf Jacobian                                   = MatrixXf::Zero(2, 2);
        
    Vector3f boundary;
    Vector3f lower_boundary_local;
    Vector3f upper_boundary_local;
    
    c_int       exitflag = 0;
    c_float     data_P_x[2];
    c_int       data_P_nnz = 2;
    c_int       data_P_i[2];
    c_int       data_P_p[2];
    c_float     data_q[2];

//    c_float     data_A_x[36];
//    c_int       data_A_nnz = 12;
//    c_int       data_A_i[36];
    
    c_float     data_A_x[2];
    c_int       data_A_nnz = 2;
    c_int       data_A_i[2];
    
    c_int       data_A_p[2];
    
    c_int       data_m;
    c_int       data_n;
    
    c_float     data_l[2];
    c_float     data_u[2];
    
    // static friction coefficient
    float mu = 0.8;

    
    Vector3f B_tangential_direction1;
    Vector3f B_tangential_direction2;
    Vector3f B_SurfaceNormal;
    
    Vector3f G_tangential_direction1;
    Vector3f G_tangential_direction2;
    Vector3f G_SurfaceNormal;
    
    VectorXf Ref_GRF                                    = VectorXf::Zero(2);
    VectorXf B_Ref_GRF                                  = VectorXf::Zero(2);
            
    VectorXf tmp_torque                                 = VectorXf::Zero(2);
    VectorXd torque                                     = VectorXd::Zero(2);
            
    MatrixXf C_                                         = MatrixXf::Zero(2, 2);

    MatrixXf C1_                                        = MatrixXf::Zero(2, 2);
    MatrixXf C2_                                        = MatrixXf::Zero(2, 2);
    MatrixXf C3_                                        = MatrixXf::Zero(2, 2);
    MatrixXf C4_                                        = MatrixXf::Zero(2, 2);
    MatrixXf C5_                                        = MatrixXf::Zero(2, 2);
    MatrixXf C6_                                        = MatrixXf::Zero(2, 2);
        
} OSQP;

class qp_practice {
public:
    // CRobot CR;
    OSQP osqp;

    void InitializeOSQP(float Robot_Mass, Vector3f FL_G_CoM2Foot_Current, Vector3f FR_G_CoM2Foot_Current, Vector3f RL_G_CoM2Foot_Current, Vector3f RR_G_CoM2Foot_Current);
    void ProcessOSQP(Vector3f FL_G_CoM2Foot_Current, Vector3f FR_G_CoM2Foot_Current, Vector3f RL_G_CoM2Foot_Current, Vector3f RR_G_CoM2Foot_Current,
                     Vector3f Foot2CoM_RefPos, Vector3f Foot2CoM_CurrentPos, Vector3f Foot2CoM_RefVel, Vector3f Foot2CoM_CurrentVel,
                     Vector3f G_Base_RefOri, Vector3f G_Base_CurrentOri, Vector3f G_Base_RefAngularVel, Vector3f G_Base_CurrentAngularVel,
                     MatrixNd Jacobian_FL, MatrixNd Jacobian_FR, MatrixNd Jacobian_RL, MatrixNd Jacobian_RR);
    void osqp_test(void);
    qp_practice();
    ~qp_practice();

    // ===================== OSQP ====================== //
    // Workspace structures
    OSQPWorkspace   *OSQP_work;
    OSQPSettings    *OSQP_settings  = (OSQPSettings *) c_malloc(sizeof (OSQPSettings));
    OSQPData        *OSQP_data      = (OSQPData *) c_malloc(sizeof (OSQPData));
    // ===================== END ====================== //
private:
};

#endif /* qp_practice_H */ 