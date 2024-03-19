/*
 * PongBot-Q Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer  : SungJoon Yoon
 * Second developer : YoungLae Cho
 * 
 */

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


//#include <chrono>   // 정밀 시간 측정을 위해 넣었음
// using namespace chrono;

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h>                              // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h>      // urdf model read using RBDL
#include <Eigen/Dense>                              // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

//* Numbers of Items and time for save //
#define SAVEDATA_LENGTH     380
#define SAVEDATA_TIME       30000

//* RCLAB CRobot Header
#include "CRobot/CRobot.h"

//* Communication Type (JoyStick)
#define SERIAL              0
#define BLUETOOTH           1
    
// Eigen Library //
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

using namespace std;

namespace gazebo 
{
    class PongBot_plugin : public ModelPlugin 
    {
        //* TIME variable       
        common::Time LastUpdatedTime;
        common::Time CurrentTime;


        //** 추가
        // common::Time calStartTime;
        // common::Time calEndTime;
        


        event::ConnectionPtr UpdateConnection;
        double dt;
        double time = 0;
        
        //* Model & Link & Joint Ptr
        physics::ModelPtr model;
        
        physics::LinkPtr BODY;
        physics::LinkPtr FL_HIP,    FR_HIP,     RL_HIP,     RR_HIP;
        physics::LinkPtr FL_THIGH,  FR_THIGH,   RL_THIGH,   RR_THIGH;
        physics::LinkPtr FL_CALF,   FR_CALF,    RL_CALF,    RR_CALF;
        physics::LinkPtr FL_TIP,    FR_TIP,     RL_TIP,     RR_TIP;
        physics::LinkPtr FL_WHEEL,  FR_WHEEL,   RL_WHEEL,   RR_WHEEL;

        physics::JointPtr FL_HR_JOINT,      FR_HR_JOINT,        RL_HR_JOINT,       RR_HR_JOINT;
        physics::JointPtr FL_HP_JOINT,      FR_HP_JOINT,        RL_HP_JOINT,       RR_HP_JOINT;
        physics::JointPtr FL_KN_JOINT,      FR_KN_JOINT,        RL_KN_JOINT,       RR_KN_JOINT;
        physics::JointPtr FL_TIP_JOINT,     FR_TIP_JOINT,       RL_TIP_JOINT,      RR_TIP_JOINT;
        physics::JointPtr FL_WHEEL_JOINT,   FR_WHEEL_JOINT,     RL_WHEEL_JOINT,    RR_WHEEL_JOINT;

        physics::JointWrench wrench;
        
        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU;
        
        //* Variables for FT sensors
        ignition::math::Vector3d torque;
        ignition::math::Vector3d force;
   
        //* Variables for save
        FILE *PongBot_data;
        int PrintDataIndex = 0;
        double PrintData[SAVEDATA_TIME][SAVEDATA_LENGTH];
        double PrintDataTime = 0.0;
        bool FileSaveFlag = false;
        bool FileSaveStopFlag = false;
        bool FileSaveDoneFlag = true;
        
        //* ROS Subscribe
        ros::NodeHandle node;
        ros::Publisher P_PrintData;
        ros::Subscriber S_JoyStick;
        std_msgs::Float64MultiArray m_PrintData;
        
        #if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d ModelPose;
            ignition::math::Vector3d BodyPose;
            ignition::math::Vector3d FL_FootPose, FR_FootPose, RL_FootPose, RR_FootPose;
            ignition::math::Vector3d BodyVel;
            ignition::math::Vector3d BodyOri;
            tf2::Quaternion BodyQuat;
        #else
            math::Pose ModelPose;
            math::Vector3 BodyPose;
            math::Vector3 BodyVel;
            math::Vector3 BodyOri;
            math::Quaternion BodyQuat;
        #endif
        
        //* CRobot Class for PongBot
        CRobot PongBot;         
        
        //* Mode & Type Index
        enum {SIMULATION = 0, ACTUAL};
        enum {LEGMODE = 0, WHEELMODE};
        
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void JointController(void);
        
        void GetLinks();
        void GetJoints();
        void GetGroundTruth();
        
        void SensorSetting();
        void IMUSensorRead();
        void FTSensorRead();
        void EncoderRead();       

        void InitJointTorque();
        
        void DataSave(void);
        void FileSave(void);
        
        void InitROSSetting();
        void ROSMsgPublish();
        void ROSJoyMode(const sensor_msgs::Joy::ConstPtr &msg);
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBot_plugin);
}

void gazebo::PongBot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {    
    Model* PONGBOT_MODEL = new Model();
    //* model.urdf file based model data input to [Model* pongbot_model] for using RBDL
    if(PongBot.Type == LEGMODE){
        //Addons::URDFReadFromFile("/home/ysj/.gazebo/models/PONGBOT_Q_V2.0/urdf/PONGBOT_Q_V2.0.urdf", PONGBOT_MODEL, true, false);
        Addons::URDFReadFromFile("/home/wonsukji/.gazebo/models/PONGBOT_Q_V2.0/urdf/PONGBOT_Q_V2.0.urdf", PONGBOT_MODEL, true, false);
    }
    else if(PongBot.Type == WHEELMODE){
        //Addons::URDFReadFromFile("/home/ysj/.gazebo/models/PONGBOT_W/urdf/PONGBOT_W.urdf", PONGBOT_MODEL, true, false);
        Addons::URDFReadFromFile("/home/wonsukji/.gazebo/models/PONGBOT_W/urdf/PONGBOT_W.urdf", PONGBOT_MODEL, true, false);
    }
        
    PongBot.setMode(SIMULATION, LEGMODE);
    PongBot.setPrintData(SAVEDATA_LENGTH, SAVEDATA_TIME);
    PongBot.setRobotModel(PONGBOT_MODEL);
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    this->model = _model;
    //* [physics::ModelPtr model] based model update
    
    GetLinks();
    GetJoints();
    
    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_MAGENTA "RBDL API version = %d\n" C_RESET, version_test);
    
    InitROSSetting();
    InitJointTorque();
    SensorSetting();
    PongBot.ControlMode         = CTRLMODE_HOME_POSE;
    PongBot.CommandFlag         = HOME_POSE;
    #if GAZEBO_MAJOR_VERSION >= 8
        this->LastUpdatedTime = this->model->GetWorld()->SimTime();
    #else
        this->LastUpdatedTime = this->model->GetWorld()->GetSimTime();
    #endif
    this->UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBot_plugin::UpdateAlgorithm, this));
}

bool testflag = true;

void gazebo::PongBot_plugin::UpdateAlgorithm() {    
    static int con_count = 0;
    
    #if GAZEBO_MAJOR_VERSION >= 8
        CurrentTime = this->model->GetWorld()->SimTime();
    #else
        CurrentTime = this->model->GetWorld()->GetSimTime();
    #endif
    dt = CurrentTime.Double() - this->LastUpdatedTime.Double();    
    time = time + dt;

    //* setting for getting dt at next step
    this->LastUpdatedTime = CurrentTime;
    
    GetGroundTruth();
    FTSensorRead();
    EncoderRead();
    IMUSensorRead();
        
    //* Real or simulated real-time thread time setting
    if(con_count % (int) (PongBot.tasktime * onesecScale) == 0){
        //* ControlMode
        switch (PongBot.ControlMode) {
            
            case CTRLMODE_HOME_POSE:
                
                break;
            
            case CTRLMODE_WALK_READY:
                
                break;
            
            case CTRLMODE_TROT:          
                
                break;         
            
            case CTRLMODE_NONE:
                
                break; 
        }
        
        //* CommandFlag
        switch (PongBot.CommandFlag){
    
            case WALK_READY:
                //PongBot.WalkingReady(4.0);  
                PongBot.WalkingReady(2.0);  
                break;
                
            case HOME_POSE:
                PongBot.HomePose(1.0);
                // std::cout << "time: " << time << std::endl;
                break;
                
            case TORQUE_OFF:
                PongBot.TorqueOff();
                break;
                
            case TROT:
                PongBot.Trot();
                
                //PongBot.MPC_controller(3, 0.002, 4);
                break;    
                
            case STAIR:
                PongBot.StairWalk();
                break;
                
            case STAND:  
                if(PongBot.ControlMode == CTRLMODE_WALK_READY){
                    PongBot.Stand();        
                }
                break;
                
            case NONE_ACT:    
                if(PongBot.ControlMode == CTRLMODE_TROT){
                    PongBot.Trot_stop();
                    std::cout << "user push stop button." << std::endl;
                }
                break;
        }
        
        
        PongBot.StateUpdate();
        if(PongBot.slope.Flag == true) {    
            PongBot.SlopeEstimation();    
        }
        if(PongBot.osqp.Flag == true){
            //PongBot.walk_para.vel_temp = 0.2;
            //PongBot.Trot();
            // if(time >= 5.){
            //     //PongBot.PatternGenerator_Walk(1., 0.1);
            // }
            
            // PongBot.ProcessOSQP();
            // if(CurrentTime > 1.1){
            //     if(static_cast<int>(time*1000) % 20 == 1){
                    
                    
            //         std::chrono::system_clock::time_point calStartTime = std::chrono::system_clock::now();
            //         PongBot.ProcessOSQP();
            //         std::chrono::system_clock::time_point calEndTime = std::chrono::system_clock::now();
            //         auto calDiffTime = std::chrono::duration_cast<std::chrono::microseconds>(calEndTime - calStartTime);
                    
                    
            //     }
            //     // std::cout << "time: " << time << std::endl;
            //     // std::cout << "speed in plugin: \n" << PongBot.base.G_GroundTruthVel << std::endl;
                
                
            // }
            // if(testflag){
            //     PongBot.walk_para_new.p_com2FL_offset <<  0.29804,  0.1988, -0.4;
            //     PongBot.walk_para_new.p_com2FR_offset <<  0.29804, -0.1988, -0.4;

            //     PongBot.walk_para_new.p_com2RL_offset << -0.29804,  0.1988, -0.4;
            //     PongBot.walk_para_new.p_com2RR_offset << -0.29804, -0.1988, -0.4;

            //     PongBot.walk_para_new.p_CoM_desired << 0., 0., 0.4;
            //     // PongBot.walk_para_new.p_CoM_desired << 0.2, 0., 0.4;

            //     PongBot.walk_para_new.p_FL_desired = PongBot.walk_para_new.p_com2FL_offset + PongBot.walk_para_new.p_CoM_desired;
            //     PongBot.walk_para_new.p_FR_desired = PongBot.walk_para_new.p_com2FR_offset + PongBot.walk_para_new.p_CoM_desired;
            //     PongBot.walk_para_new.p_RL_desired = PongBot.walk_para_new.p_com2RL_offset + PongBot.walk_para_new.p_CoM_desired;
            //     PongBot.walk_para_new.p_RR_desired = PongBot.walk_para_new.p_com2RR_offset + PongBot.walk_para_new.p_CoM_desired;

            //     testflag = false;
            // }


            // PongBot.walk_para_new.gait_type_target = MOVE_WALK;
            // PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.;
            

            if(time > 5.){


                if(testflag){
                    PongBot.walk_para_new.p_com2FL_offset = PongBot.FL_Foot.B_RefPos;
                    PongBot.walk_para_new.p_com2FR_offset = PongBot.FR_Foot.B_RefPos;

                    PongBot.walk_para_new.p_com2RL_offset = PongBot.RL_Foot.B_RefPos;
                    PongBot.walk_para_new.p_com2RR_offset = PongBot.RR_Foot.B_RefPos;

                    PongBot.walk_para_new.p_CoM_desired << 0., 0., 0.4;


                    // PongBot.walk_para_new.p_CoM_desired << 0.2, 0., 0.4;

                    PongBot.walk_para_new.p_FL_desired = PongBot.walk_para_new.p_com2FL_offset + PongBot.walk_para_new.p_CoM_desired;
                    PongBot.walk_para_new.p_FR_desired = PongBot.walk_para_new.p_com2FR_offset + PongBot.walk_para_new.p_CoM_desired;
                    PongBot.walk_para_new.p_RL_desired = PongBot.walk_para_new.p_com2RL_offset + PongBot.walk_para_new.p_CoM_desired;
                    PongBot.walk_para_new.p_RR_desired = PongBot.walk_para_new.p_com2RR_offset + PongBot.walk_para_new.p_CoM_desired;

                    testflag = false;

                    // PongBot.walk_para_new.gait_type_target = MOVE_WALK;
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.05;
                }

                if(time > 13. && time <= 25.){
                    // PongBot.walk_para_new.gait_type_target = MOVE_WALK;
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.1;
                }
                else if(time > 25. && time <= 35.){
                    // PongBot.walk_para_new.gait_type_target = MOVE_WALK;
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.1;
                }

                else if(time > 35. && time <= 42.){
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    // PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.2;
                }

                else if (time > 42. && time < 48. ){
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    // PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.4;
                }

                else if (time > 48. && time < 54. ){
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    // PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.6;
                }

                else if (time > 54. && time < 60. ){
                    PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    // PongBot.walk_para_new.gait_type_target = MOVE_TROT;
                    PongBot.walk_para_new.v_CoM_joystick(AXIS_X) = 0.8;
                }

                PongBot.Timer_for_gait_traj(PongBot.walk_para_new.dt_gait, PongBot.walk_para_new.t_p, PongBot.walk_para_new.time_gait);
                
                // std::cout << "time_gait: " <<  PongBot.walk_para_new.time_gait << std::endl;
                if(PongBot.walk_para_new.time_gait == PongBot.walk_para_new.dt_gait){
                    PongBot.walk_para_new.gait_phase = STANCE_PHASE;
                    // std::cout << "STANCE_PHASE" << std::endl;
                    // std::cout << "t_stance: " << PongBot.walk_para_new.t_stance << std::endl;
                    // std::cout << "t_swing: " << PongBot.walk_para_new.t_swing << std::endl;

                    PongBot.PrevDataSave(&PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_desired,
                                         PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired, PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired,
                                         PongBot.walk_para_new.v_CoM_desired, &PongBot.walk_para_new.p_CoM_prev,
                                         &PongBot.walk_para_new.p_FL_now, &PongBot.walk_para_new.p_FR_now, &PongBot.walk_para_new.p_RL_now, &PongBot.walk_para_new.p_RR_now,
                                         &PongBot.walk_para_new.v_CoM_prev);
                    
                    PongBot.Gait_Selector(PongBot.walk_para_new.gait_type_target, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_type_changed);

                    
                    PongBot.SwingLegChecker(PongBot.walk_para_new.gait_type, PongBot.walk_para_new.swingLeg);

                    
                    PongBot.Optimization_Process(PongBot.walk_para_new.gait_type, PongBot.walk_para_new.v_CoM_joystick,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing,
                                                 &PongBot.walk_para_new.v_CoM_desired);


                    PongBot.walk_para_new.p_CoM_desired = PongBot.CoM_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing,
                                                                                        PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                        PongBot.walk_para_new.gait_type_changed, PongBot.walk_para_new.swingLeg,
                                                                                        PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_prev,
                                                                                        PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired, PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired);

                }
                else if( round(1000.*PongBot.walk_para_new.time_gait) == round(1000.*(PongBot.walk_para_new.dt_gait + PongBot.walk_para_new.t_stance))){
                    PongBot.walk_para_new.gait_phase = SWING_PHASE;
                    // std::cout << "SWING_PHASE" << std::endl;

                    PongBot.PrevDataSave(&PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_desired,
                                         PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired, PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired,
                                         PongBot.walk_para_new.v_CoM_desired, &PongBot.walk_para_new.p_CoM_prev,
                                         &PongBot.walk_para_new.p_FL_now, &PongBot.walk_para_new.p_FR_now, &PongBot.walk_para_new.p_RL_now, &PongBot.walk_para_new.p_RR_now,
                                         &PongBot.walk_para_new.v_CoM_prev);

                    PongBot.walk_para_new.p_CoM_desired = PongBot.CoM_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing,
                                                                                        PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                        PongBot.walk_para_new.gait_type_changed, PongBot.walk_para_new.swingLeg,
                                                                                        PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_prev,
                                                                                        PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired, PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired);

                    if(PongBot.walk_para_new.gait_type == MOVE_WALK){
                        if(PongBot.walk_para_new.swingLeg == LEGPHASE_FL){
                            
                            PongBot.walk_para_new.p_FL_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2FL_offset);
                        }
                        else if(PongBot.walk_para_new.swingLeg == LEGPHASE_FR){
                            PongBot.walk_para_new.p_FR_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_FR_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2FR_offset);
                        }
                        else if(PongBot.walk_para_new.swingLeg == LEGPHASE_RL){
                            PongBot.walk_para_new.p_RL_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2RL_offset);
                        }
                        else if(PongBot.walk_para_new.swingLeg == LEGPHASE_RR){
                            PongBot.walk_para_new.p_RR_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2RR_offset);
                        }
                    }
                    else if(PongBot.walk_para_new.gait_type == MOVE_TROT){
                        if(PongBot.walk_para_new.swingLeg == LEGPHASE_FLRR){
                            PongBot.walk_para_new.p_FL_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2FL_offset);
                            
                            PongBot.walk_para_new.p_RR_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2RR_offset);
                        }
                        else if(PongBot.walk_para_new.swingLeg == LEGPHASE_FRRL){
                            PongBot.walk_para_new.p_FR_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_FR_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2FR_offset);
                            PongBot.walk_para_new.p_RL_desired = PongBot.Foot_EndPoint_Generator(PongBot.walk_para_new.t_p, PongBot.walk_para_new.t_stance, PongBot.walk_para_new.v_CoM_desired,
                                                                                         PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                                                                         PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.p_com2RL_offset);
                        }
                    }

                }

                // PongBot.walk_para_new.p_CoM_traj = PongBot.CoM_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                //                 PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.v_CoM_prev, PongBot.walk_para_new.p_CoM_desired,
                //                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, PongBot.walk_para_new.time_gait, PongBot.walk_para_new.dt_gait);


                // p_CoM_traj 생성
                PongBot.walk_para_new.p_CoM_traj = PongBot.CoM_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase,
                                PongBot.walk_para_new.p_CoM_now, PongBot.walk_para_new.p_CoM_desired, PongBot.walk_para_new.v_CoM_prev, PongBot.walk_para_new.v_CoM_desired,
                                PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, PongBot.walk_para_new.time_gait, PongBot.walk_para_new.dt_gait);
                                //PongBot.walk_para_new.CoM_traj_x_5thpoly_coeffs, PongBot.walk_para_new.CoM_traj_y_5thpoly_coeffs);


                // p_FL_traj, p_FR_traj, p_RL_traj, p_RR_traj 생성
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, PongBot.walk_para_new.time_gait, PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_traj, &PongBot.walk_para_new.p_FR_traj, &PongBot.walk_para_new.p_RL_traj, &PongBot.walk_para_new.p_RR_traj);


                // 미래의 다리 궤적들을 여기서 생성
                // 1. 40ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.040)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future1_traj, &PongBot.walk_para_new.p_FR_future1_traj, &PongBot.walk_para_new.p_RL_future1_traj, &PongBot.walk_para_new.p_RR_future1_traj);

                // 2. 80ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.080)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future2_traj, &PongBot.walk_para_new.p_FR_future2_traj, &PongBot.walk_para_new.p_RL_future2_traj, &PongBot.walk_para_new.p_RR_future2_traj);

                // 3. 120ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.120)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future3_traj, &PongBot.walk_para_new.p_FR_future3_traj, &PongBot.walk_para_new.p_RL_future3_traj, &PongBot.walk_para_new.p_RR_future3_traj);

                // 4. 160ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.160)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future4_traj, &PongBot.walk_para_new.p_FR_future4_traj, &PongBot.walk_para_new.p_RL_future4_traj, &PongBot.walk_para_new.p_RR_future4_traj);

                // 5. 200ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.200)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future5_traj, &PongBot.walk_para_new.p_FR_future5_traj, &PongBot.walk_para_new.p_RL_future5_traj, &PongBot.walk_para_new.p_RR_future5_traj);

                // 6. 240ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.240)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future6_traj, &PongBot.walk_para_new.p_FR_future6_traj, &PongBot.walk_para_new.p_RL_future6_traj, &PongBot.walk_para_new.p_RR_future6_traj);

                // 7. 280ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.280)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future7_traj, &PongBot.walk_para_new.p_FR_future7_traj, &PongBot.walk_para_new.p_RL_future7_traj, &PongBot.walk_para_new.p_RR_future7_traj);

                // 8. 320ms 미래
                PongBot.Foot_Traj_Generator(PongBot.walk_para_new.gait_on_off_now, PongBot.walk_para_new.gait_type, PongBot.walk_para_new.gait_phase, PongBot.walk_para_new.swingLeg,
                                                 PongBot.walk_para_new.p_RL_now, PongBot.walk_para_new.p_RR_now, PongBot.walk_para_new.p_FL_now, PongBot.walk_para_new.p_FR_now,
                                                 PongBot.walk_para_new.p_RL_desired, PongBot.walk_para_new.p_RR_desired, PongBot.walk_para_new.p_FL_desired, PongBot.walk_para_new.p_FR_desired,
                                                 PongBot.walk_para_new.t_stance, PongBot.walk_para_new.t_swing, round((PongBot.walk_para_new.time_gait + 0.320)*1000.)/1000., PongBot.walk_para_new.foot_traj_height,
                                                 PongBot.walk_para_new.t_p, PongBot.walk_para_new.v_CoM_desired, PongBot.walk_para_new.p_CoM_desired,
                                                 &PongBot.walk_para_new.p_FL_future8_traj, &PongBot.walk_para_new.p_FR_future8_traj, &PongBot.walk_para_new.p_RL_future8_traj, &PongBot.walk_para_new.p_RR_future8_traj);










                
                Vector12f RefPos_B;
                Vector12f Joint_RefPos;
                RefPos_B << PongBot.walk_para_new.p_FL_traj - PongBot.walk_para_new.p_CoM_traj,
                            PongBot.walk_para_new.p_FR_traj - PongBot.walk_para_new.p_CoM_traj,
                            PongBot.walk_para_new.p_RL_traj - PongBot.walk_para_new.p_CoM_traj,
                            PongBot.walk_para_new.p_RR_traj - PongBot.walk_para_new.p_CoM_traj;
                
                
                
                //EP_InitPos                                  << FL_Foot.B_RefPos, FR_Foot.B_RefPos, RL_Foot.B_RefPos, RR_Foot.B_RefPos;
                PongBot.FL_Foot.B_RefPos = PongBot.walk_para_new.p_FL_traj - PongBot.walk_para_new.p_CoM_traj;
                PongBot.FR_Foot.B_RefPos = PongBot.walk_para_new.p_FR_traj - PongBot.walk_para_new.p_CoM_traj;
                PongBot.RL_Foot.B_RefPos = PongBot.walk_para_new.p_RL_traj - PongBot.walk_para_new.p_CoM_traj;
                PongBot.RR_Foot.B_RefPos = PongBot.walk_para_new.p_RR_traj - PongBot.walk_para_new.p_CoM_traj;


                Joint_RefPos                              = PongBot.computeIK(RefPos_B);
                
                for (int nJoint = 0; nJoint < 12; ++nJoint) {
                        PongBot.joint[nJoint].RefPos            = Joint_RefPos[nJoint];

                        if(nJoint == 2){
                            // std::cout << PongBot.joint[nJoint].RefPos << std::endl;
                        }
                }


                // Vector3f temp_vector = VectorXf::Zero(3);
                // std::cout << "temp_vector(before): " << temp_vector << std::endl;
                // PongBot.Test_vectorxf_reference(&temp_vector);
                // std::cout << "temp_vector(after): " << temp_vector << std::endl;


                // std::cout << PongBot.FL_Foot.G_RefPos << std::endl;
                // std::cout << PongBot.RL_Foot.G_RefPos << std::endl;
                // std::cout << dt << std::endl;


            } // end of time=5.;

        }
        if(PongBot.torque.CTC_Flag == true){
            PongBot.ComputeTorqueControl();
        }

        // ============= DATASAVE ============= //
        if(FileSaveFlag == true){
            DataSave();
        }
        
        ROSMsgPublish();
        FileSave();
        
        con_count = 0;
    }

    JointController();
    con_count++;
}

void gazebo::PongBot_plugin::GetLinks() {
    /* GetLinks
     * Load Link Information(GAZEBO)
     */
    this->BODY      = this->model->GetLink("BODY");
    
    this->FL_HIP    = this->model->GetLink("FL_HIP");
    this->FL_THIGH  = this->model->GetLink("FL_THIGH");
    this->FL_CALF   = this->model->GetLink("FL_CALF");
    this->FR_HIP    = this->model->GetLink("FR_HIP");
    this->FR_THIGH  = this->model->GetLink("FR_THIGH");
    this->FR_CALF   = this->model->GetLink("FR_CALF");
    this->RL_HIP    = this->model->GetLink("RL_HIP");
    this->RL_THIGH  = this->model->GetLink("RL_THIGH");
    this->RL_CALF   = this->model->GetLink("RL_CALF");
    this->RR_HIP    = this->model->GetLink("RR_HIP");
    this->RR_THIGH  = this->model->GetLink("RR_THIGH");
    this->RR_CALF   = this->model->GetLink("RR_CALF");
    
    if(PongBot.Type == LEGMODE){
        this->FL_TIP    = this->model->GetLink("FL_TIP");
        this->FR_TIP    = this->model->GetLink("FR_TIP");
        this->RL_TIP    = this->model->GetLink("RL_TIP");
        this->RR_TIP    = this->model->GetLink("RR_TIP");
    }
    else if(PongBot.Type == WHEELMODE){
        this->FL_WHEEL  = this->model->GetLink("FL_WHEEL");
        this->FR_WHEEL  = this->model->GetLink("FR_WHEEL");
        this->RL_WHEEL  = this->model->GetLink("RL_WHEEL");
        this->RR_WHEEL  = this->model->GetLink("RR_WHEEL");
    }    
}

void gazebo::PongBot_plugin::GetJoints() {
    /* GetJoints
     * Load Joint Information(GAZEBO) 
     */
    this->FL_HR_JOINT   = this->model->GetJoint("FL_HR_JOINT");
    this->FL_HP_JOINT   = this->model->GetJoint("FL_HP_JOINT");
    this->FL_KN_JOINT   = this->model->GetJoint("FL_KN_JOINT");
    this->FR_HR_JOINT   = this->model->GetJoint("FR_HR_JOINT");
    this->FR_HP_JOINT   = this->model->GetJoint("FR_HP_JOINT");
    this->FR_KN_JOINT   = this->model->GetJoint("FR_KN_JOINT");
    this->RL_HR_JOINT   = this->model->GetJoint("RL_HR_JOINT");
    this->RL_HP_JOINT   = this->model->GetJoint("RL_HP_JOINT");
    this->RL_KN_JOINT   = this->model->GetJoint("RL_KN_JOINT");
    this->RR_HR_JOINT   = this->model->GetJoint("RR_HR_JOINT");
    this->RR_HP_JOINT   = this->model->GetJoint("RR_HP_JOINT");
    this->RR_KN_JOINT   = this->model->GetJoint("RR_KN_JOINT");

    if(PongBot.Type == LEGMODE){
        this->FL_TIP_JOINT  = this->model->GetJoint("FL_TIP_JOINT");
        this->FR_TIP_JOINT  = this->model->GetJoint("FR_TIP_JOINT");
        this->RL_TIP_JOINT  = this->model->GetJoint("RL_TIP_JOINT");
        this->RR_TIP_JOINT  = this->model->GetJoint("RR_TIP_JOINT");
    }
    else if(PongBot.Type == WHEELMODE){
        this->FL_WHEEL_JOINT  = this->model->GetJoint("FL_WHEEL_JOINT");
        this->FR_WHEEL_JOINT  = this->model->GetJoint("FR_WHEEL_JOINT");
        this->RL_WHEEL_JOINT  = this->model->GetJoint("RL_WHEEL_JOINT");
        this->RR_WHEEL_JOINT  = this->model->GetJoint("RR_WHEEL_JOINT");
    }    
}

void gazebo::PongBot_plugin::InitROSSetting() {  
    /* InitROSSetting
     * Initialize ROS Setting
     */
    S_JoyStick = node.subscribe("/joy", 1, &gazebo::PongBot_plugin::ROSJoyMode, this);
    P_PrintData = node.advertise<std_msgs::Float64MultiArray>("/pongbot_data/", 1000);    
    m_PrintData.data.resize(PongBot.print.DataLength);
}


void gazebo::PongBot_plugin::SensorSetting() {
    /* SensorSetting
     * Initialize IMU Setting(GAZEBO)
     */
    this->Sensor = sensors::get_sensor("IMU");
    this->IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
}

void gazebo::PongBot_plugin::EncoderRead() {
    /* EncoderRead
     * Load Encoder Information(GAZEBO)
     */

    #if GAZEBO_MAJOR_VERSION >= 8
        PongBot.joint[PongBot.FLHR].CurrentPos        = this->FL_HR_JOINT->Position(1);
        PongBot.joint[PongBot.FLHP].CurrentPos        = this->FL_HP_JOINT->Position(1);
        PongBot.joint[PongBot.FLKN].CurrentPos        = this->FL_KN_JOINT->Position(1);

        PongBot.joint[PongBot.FRHR].CurrentPos        = this->FR_HR_JOINT->Position(1);
        PongBot.joint[PongBot.FRHP].CurrentPos        = this->FR_HP_JOINT->Position(1);
        PongBot.joint[PongBot.FRKN].CurrentPos        = this->FR_KN_JOINT->Position(1);
        
        PongBot.joint[PongBot.RLHR].CurrentPos        = this->RL_HR_JOINT->Position(1);
        PongBot.joint[PongBot.RLHP].CurrentPos        = this->RL_HP_JOINT->Position(1);
        PongBot.joint[PongBot.RLKN].CurrentPos        = this->RL_KN_JOINT->Position(1);

        PongBot.joint[PongBot.RRHR].CurrentPos        = this->RR_HR_JOINT->Position(1);
        PongBot.joint[PongBot.RRHP].CurrentPos        = this->RR_HP_JOINT->Position(1);
        PongBot.joint[PongBot.RRKN].CurrentPos        = this->RR_KN_JOINT->Position(1);
        
        if(PongBot.Type == WHEELMODE){
            PongBot.joint[PongBot.FLWHL].CurrentPos     = this->RR_HR_JOINT->Position(1);
            PongBot.joint[PongBot.FRWHL].CurrentPos     = this->RR_HR_JOINT->Position(1);
            PongBot.joint[PongBot.RLWHL].CurrentPos     = this->RR_HR_JOINT->Position(1);
            PongBot.joint[PongBot.RRWHL].CurrentPos     = this->RR_HR_JOINT->Position(1);
        }
    #else
        PongBot.joint[PongBot.FLHR].CurrentPos = this->FL_HR_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.FLHP].CurrentPos = this->FL_HP_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.FLKN].CurrentPos = this->FL_KN_JOINT->GetAngle(0).Radian();

        PongBot.joint[PongBot.FRHR].CurrentPos = this->FR_HR_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.FRHP].CurrentPos = this->FR_HP_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.FRKN].CurrentPos = this->FR_KN_JOINT->GetAngle(0).Radian();
        
        PongBot.joint[PongBot.RLHR].CurrentPos = this->RL_HR_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.RLHP].CurrentPos = this->RL_HP_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.RLKN].CurrentPos = this->RL_KN_JOINT->GetAngle(0).Radian();

        PongBot.joint[PongBot.RRHR].CurrentPos = this->RR_HR_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.RRHP].CurrentPos = this->RR_HP_JOINT->GetAngle(0).Radian();
        PongBot.joint[PongBot.RRKN].CurrentPos = this->RR_KN_JOINT->GetAngle(0).Radian();
        
        if(PongBot.Type == WHEELMODE){
            PongBot.joint[PongBot.FLWHL].CurrentPos  = this->FL_WHEEL_JOINT->GetAngle(0).Radian();
            PongBot.joint[PongBot.FRWHL].CurrentPos  = this->FR_WHEEL_JOINT->GetAngle(0).Radian();
            PongBot.joint[PongBot.RLWHL].CurrentPos  = this->RL_WHEEL_JOINT->GetAngle(0).Radian();
            PongBot.joint[PongBot.RRWHL].CurrentPos  = this->RR_WHEEL_JOINT->GetAngle(0).Radian();
        }
        
    #endif
        PongBot.joint[PongBot.FLHR].CurrentVel  = this->FL_HR_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.FLHP].CurrentVel  = this->FL_HP_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.FLKN].CurrentVel  = this->FL_KN_JOINT->GetVelocity(0); 

        PongBot.joint[PongBot.FRHR].CurrentVel  = this->FR_HR_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.FRHP].CurrentVel  = this->FR_HP_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.FRKN].CurrentVel  = this->FR_KN_JOINT->GetVelocity(0);
        
        PongBot.joint[PongBot.RLHR].CurrentVel  = this->RL_HR_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.RLHP].CurrentVel  = this->RL_HP_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.RLKN].CurrentVel  = this->RL_KN_JOINT->GetVelocity(0);
    
        PongBot.joint[PongBot.RRHR].CurrentVel  = this->RR_HR_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.RRHP].CurrentVel  = this->RR_HP_JOINT->GetVelocity(0);
        PongBot.joint[PongBot.RRKN].CurrentVel  = this->RR_KN_JOINT->GetVelocity(0);

        if(PongBot.Type == WHEELMODE){
            PongBot.joint[PongBot.FLWHL].CurrentVel  = this->FL_WHEEL_JOINT->GetVelocity(0);
            PongBot.joint[PongBot.FRWHL].CurrentVel  = this->FR_WHEEL_JOINT->GetVelocity(0);
            PongBot.joint[PongBot.RLWHL].CurrentVel  = this->RL_WHEEL_JOINT->GetVelocity(0);
            PongBot.joint[PongBot.RRWHL].CurrentVel  = this->RR_WHEEL_JOINT->GetVelocity(0);
        }
        
        PongBot.joint[PongBot.FLHR].CurrentTorque  = this->FL_HR_JOINT->GetForce(0);
        PongBot.joint[PongBot.FLHP].CurrentTorque  = this->FL_HP_JOINT->GetForce(0);
        PongBot.joint[PongBot.FLKN].CurrentTorque  = this->FL_KN_JOINT->GetForce(0); 
        
        PongBot.joint[PongBot.FRHR].CurrentTorque  = this->FR_HR_JOINT->GetForce(0);
        PongBot.joint[PongBot.FRHP].CurrentTorque  = this->FR_HP_JOINT->GetForce(0);
        PongBot.joint[PongBot.FRKN].CurrentTorque  = this->FR_KN_JOINT->GetForce(0); 
        
        PongBot.joint[PongBot.RLHR].CurrentTorque  = this->RL_HR_JOINT->GetForce(0);
        PongBot.joint[PongBot.RLHP].CurrentTorque  = this->RL_HP_JOINT->GetForce(0);
        PongBot.joint[PongBot.RLKN].CurrentTorque  = this->RL_KN_JOINT->GetForce(0); 
        
        PongBot.joint[PongBot.RRHR].CurrentTorque  = this->RR_HR_JOINT->GetForce(0);
        PongBot.joint[PongBot.RRHP].CurrentTorque  = this->RR_HP_JOINT->GetForce(0);
        PongBot.joint[PongBot.RRKN].CurrentTorque  = this->RR_KN_JOINT->GetForce(0); 
}

void gazebo::PongBot_plugin::InitJointTorque() {
    /* InitJointTorque
     * Initialize Joint Torque
     */
    for (int nJoint = 0; nJoint < JOINT_NUM; ++nJoint) {
        PongBot.joint[nJoint].RefTorque    = 0.0;
    }
}


void gazebo::PongBot_plugin::JointController(void) {
    /* JointController
     * Control Joint Torque
     */
    this->RL_HR_JOINT->SetForce(0, PongBot.joint[PongBot.RLHR].RefTorque);
    this->RL_HP_JOINT->SetForce(0, PongBot.joint[PongBot.RLHP].RefTorque);
    this->RL_KN_JOINT->SetForce(0, PongBot.joint[PongBot.RLKN].RefTorque);
    
    this->RR_HR_JOINT->SetForce(0, PongBot.joint[PongBot.RRHR].RefTorque);
    this->RR_HP_JOINT->SetForce(0, PongBot.joint[PongBot.RRHP].RefTorque);
    this->RR_KN_JOINT->SetForce(0, PongBot.joint[PongBot.RRKN].RefTorque);
    
    this->FL_HR_JOINT->SetForce(0, PongBot.joint[PongBot.FLHR].RefTorque);
    this->FL_HP_JOINT->SetForce(0, PongBot.joint[PongBot.FLHP].RefTorque);
    this->FL_KN_JOINT->SetForce(0, PongBot.joint[PongBot.FLKN].RefTorque);

    this->FR_HR_JOINT->SetForce(0, PongBot.joint[PongBot.FRHR].RefTorque);
    this->FR_HP_JOINT->SetForce(0, PongBot.joint[PongBot.FRHP].RefTorque);
    this->FR_KN_JOINT->SetForce(0, PongBot.joint[PongBot.FRKN].RefTorque);
        
    if(PongBot.Type == WHEELMODE){
        // SetVelocity : Radian/sec
        this->FL_WHEEL_JOINT->SetVelocity(0, PongBot.wheel.Velocity);
        this->FR_WHEEL_JOINT->SetVelocity(0, PongBot.wheel.Velocity);
        this->RL_WHEEL_JOINT->SetVelocity(0, PongBot.wheel.Velocity);
        this->RR_WHEEL_JOINT->SetVelocity(0, PongBot.wheel.Velocity);
    }
}


void gazebo::PongBot_plugin::ROSMsgPublish() {
    /* ROSMsgPublish
     * Publish ROS topic & msg 
     */
    for (unsigned int i = 0; i < PongBot.print.DataLength; ++i) {
        m_PrintData.data[i] = PongBot.print.Data(i);
    }
        P_PrintData.publish(m_PrintData);
}

void gazebo::PongBot_plugin::IMUSensorRead() {
    // IMU sensor data    
    static double IMU_CurrentAngle[3], IMU_CurrentAngularVel[3], IMU_CurrentLinearAcc[3];
    
    #if GAZEBO_MAJOR_VERSION >= 8
        IMU_CurrentAngularVel[AXIS_ROLL]    = this->IMU->AngularVelocity(false)[AXIS_ROLL];
        IMU_CurrentAngularVel[AXIS_PITCH]   = this->IMU->AngularVelocity(false)[AXIS_PITCH];
        IMU_CurrentAngularVel[AXIS_YAW]     = this->IMU->AngularVelocity(false)[AXIS_YAW];
        
        IMU_CurrentLinearAcc[AXIS_X]        = this->IMU->LinearAcceleration()[AXIS_X];
        IMU_CurrentLinearAcc[AXIS_Y]        = this->IMU->LinearAcceleration()[AXIS_Y];
        IMU_CurrentLinearAcc[AXIS_Z]        = this->IMU->LinearAcceleration()[AXIS_Z];
        
        IMU_CurrentAngle[AXIS_ROLL]         = this->IMU->Orientation().Euler()[AXIS_ROLL];
        IMU_CurrentAngle[AXIS_PITCH]        = this->IMU->Orientation().Euler()[AXIS_PITCH];
        IMU_CurrentAngle[AXIS_YAW]          = this->IMU->Orientation().Euler()[AXIS_YAW];
        
    #else
        IMU_CurrentAngularVel[AXIS_ROLL]    = this->IMU->AngularVelocity(false)[AXIS_ROLL];
        IMU_CurrentAngularVel[AXIS_PITCH]   = this->IMU->AngularVelocity(false)[AXIS_PITCH];
        IMU_CurrentAngularVel[AXIS_YAW]     = this->IMU->AngularVelocity(false)[AXIS_YAW];

        IMU_CurrentAngle[AXIS_ROLL]         = this->IMU->Orientation().Euler()[AXIS_ROLL];
        IMU_CurrentAngle[AXIS_PITCH]        = this->IMU->Orientation().Euler()[AXIS_PITCH];
        IMU_CurrentAngle[AXIS_YAW]          = this->IMU->Orientation().Euler()[AXIS_YAW];
    #endif
        
    PongBot.imu.AngularVel[AXIS_ROLL]       = IMU_CurrentAngularVel[AXIS_ROLL];
    PongBot.imu.AngularVel[AXIS_PITCH]      = IMU_CurrentAngularVel[AXIS_PITCH];
    PongBot.imu.AngularVel[AXIS_YAW]        = IMU_CurrentAngularVel[AXIS_YAW];

    PongBot.imu.LinearAcc[AXIS_X]           = IMU_CurrentLinearAcc[AXIS_X];
    PongBot.imu.LinearAcc[AXIS_Y]           = IMU_CurrentLinearAcc[AXIS_Y];
    PongBot.imu.LinearAcc[AXIS_Z]           = IMU_CurrentLinearAcc[AXIS_Z];
    
    PongBot.imu.TempOri_ZYX(AXIS_ROLL)      = IMU_CurrentAngle[AXIS_ROLL];
    PongBot.imu.TempOri_ZYX(AXIS_PITCH)     = IMU_CurrentAngle[AXIS_PITCH];
    PongBot.imu.TempOri_ZYX(AXIS_YAW)       = IMU_CurrentAngle[AXIS_YAW];            
}


void gazebo::PongBot_plugin::ROSJoyMode(const sensor_msgs::Joy::ConstPtr &msg) {    
   /* ROSJoyMode
    * Joystick Mode
    * 
    * Button information
    * WalkReady     : [Board button]
    * Trot          : [Square button]
    * Pronk         : [Triangle button]
    * Flying Trot   : [R1 button]
    * Move Stop     : [Circle button]
    * Data Save     : [L1 button]
    * Data Save Stop: [L2 button]     
    * TorqueOff     : [Option button]
    */
    
    static bool JoyMode         = 0.0;    
    static bool ActiveFlag      = 0.0;
        
    if(msg->buttons.size() == 13){JoyMode = SERIAL;}
    else{JoyMode = BLUETOOTH;}
    
    if(JoyMode == BLUETOOTH)
    {
        if (msg->buttons[0] == true) {
            if((PongBot.walking.Ready == true)&&(PongBot.walking.MoveDone == true)){
                PongBot.ControlMode             = CTRLMODE_TROT;
                PongBot.CommandFlag             = TROT;
            }
        }
        else if (msg->buttons[1] == true) {
            if((PongBot.walking.Ready == true)&&(PongBot.walking.MoveDone == true)){
                PongBot.ControlMode             = CTRLMODE_WALK_READY;
                PongBot.CommandFlag             = STAND;

                PongBot.standturn.Startflag     = true;
            }
        }
        else if (msg->buttons[2] == true) {
            if (PongBot.walking.Stop == false){
                PongBot.walking.Stop = true;
            }
        }
        else if (msg->buttons[4] == true) {
            if(FileSaveDoneFlag == true){
                FileSaveFlag = true;
                FileSaveDoneFlag = false;
            };
        }
        else if (msg->buttons[8] == true) {
            if(FileSaveStopFlag == false){
                FileSaveStopFlag = true;
            };
        }
        else if (msg->buttons[9] == true) {
            PongBot.ControlMode                 = CTRLMODE_NONE;
            PongBot.CommandFlag                 = TORQUE_OFF;
            FileSaveStopFlag                    = true;
        }
        else if (msg->buttons[13] == true) {
            PongBot.ControlMode                 = CTRLMODE_HOME_POSE;
            PongBot.CommandFlag                 = HOME_POSE;
        }

        PongBot.walking.InputData[AXIS_X]               = (msg->axes[1]);
        PongBot.walking.InputData[AXIS_Y]               = (msg->axes[0]);
        PongBot.walking.InputData[AXIS_YAW]             = (msg->axes[2]);

        if(msg->axes[3] > 0){
            ActiveFlag = 1;
        }

        if(ActiveFlag == 0){
            PongBot.walking.CoM_InputData[AXIS_Z]       = 1;
        }
        else{
            PongBot.walking.CoM_InputData[AXIS_Z]       = (msg->axes[3]);
        }

        if((msg->buttons[5] == true)&&(PongBot.CommandFlag == STAND)){
        	PongBot.wheel.Velocity = PongBot.wheel.SpeedWeight * (msg->axes[5]);
        }
        else{
            PongBot.wheel.Velocity = 0;
        }
    }
    
    if(JoyMode == SERIAL)
    {
        if (msg->buttons[3] == true) {
            if((PongBot.walking.Ready == true)&&(PongBot.walking.MoveDone == true)){
                PongBot.ControlMode             = CTRLMODE_TROT;
                PongBot.CommandFlag             = TROT;
            }
        }
        else if (msg->buttons[0] == true) {
            if((PongBot.walking.Ready == true)&&(PongBot.walking.MoveDone == true)){
                PongBot.ControlMode             = CTRLMODE_WALK_READY;
                PongBot.CommandFlag             = STAND;
            
                PongBot.standturn.Startflag     = true;
            }
        }
        else if (msg->buttons[1] == true) {       
            if (PongBot.walking.Stop == false){
                PongBot.walking.Stop = true;
            }
        }
        else if (msg->buttons[4] == true) {
            if(FileSaveDoneFlag == true){
                FileSaveFlag = true;
                FileSaveDoneFlag = false;
            };   
        }
        else if (msg->buttons[6] == true) {
            if(FileSaveStopFlag == false){
                FileSaveStopFlag = true;
            };   
        }
        else if (msg->buttons[9] == true) {
            PongBot.ControlMode                 = CTRLMODE_NONE;
            PongBot.CommandFlag                 = TORQUE_OFF;
            FileSaveStopFlag                    = true;
        }
        
        PongBot.walking.InputData[AXIS_X]       = (msg->axes[1]);
        PongBot.walking.InputData[AXIS_Y]       = (msg->axes[0]);
        PongBot.walking.InputData[AXIS_YAW]     = (msg->axes[3]);
              
        if(msg->axes[2] > 0){
            ActiveFlag = 1;
        }
        
        if(ActiveFlag == 0){
            PongBot.walking.CoM_InputData[AXIS_Z]       = 1;
        }
        else{
            PongBot.walking.CoM_InputData[AXIS_Z]       = (msg->axes[2]);
        }
        
        if((msg->buttons[5] == true)&&(PongBot.CommandFlag == STAND)){
        	PongBot.wheel.Velocity = PongBot.wheel.SpeedWeight * (msg->axes[4]);
        }
        else{
            PongBot.wheel.Velocity = 0;
        }
    }
}

void gazebo::PongBot_plugin::FTSensorRead() {
    /* FTSensorRead(Not Use)
     */
    
    static int RL_Fz_ReadCount = 0, RR_Fz_ReadCount = 0, FL_Fz_ReadCount = 0, FR_Fz_ReadCount = 0;
    
    const int ReadCountThreshold    = 5;
    const double ForceThreshold     = 50;
    
    if(PongBot.Type == LEGMODE){
        wrench = this->FL_TIP_JOINT->GetForceTorque(0);
    }
    else if(PongBot.Type == WHEELMODE){
        wrench = this->FL_WHEEL_JOINT->GetForceTorque(0);
    } 
    
    #if GAZEBO_MAJOR_VERSION >= 8
        force = wrench.body2Force;
        torque = wrench.body2Torque;
    #else
        force = wrench.body2Force.Ign();
        torque = wrench.body2Torque.Ign();
    #endif    
    
    PongBot.FL_Foot.FTSensor(PongBot.Fx) = force.X();
    PongBot.FL_Foot.FTSensor(PongBot.Fy) = force.Y();
    PongBot.FL_Foot.FTSensor(PongBot.Fz) = force.Z();
    
    
    if(PongBot.Type == LEGMODE){
        wrench = this->FR_TIP_JOINT->GetForceTorque(0);
    }
    else if(PongBot.Type == WHEELMODE){
        wrench = this->FR_WHEEL_JOINT->GetForceTorque(0);
    }
    
    #if GAZEBO_MAJOR_VERSION >= 8
        force = wrench.body2Force;
        torque = wrench.body2Torque;
    #else
        force = wrench.body2Force.Ign();
        torque = wrench.body2Torque.Ign();
    #endif    
    
    PongBot.FR_Foot.FTSensor(PongBot.Fx) = force.X();
    PongBot.FR_Foot.FTSensor(PongBot.Fy) = force.Y();
    PongBot.FR_Foot.FTSensor(PongBot.Fz) = force.Z();
    
    if(PongBot.Type == LEGMODE){
        wrench = this->RL_TIP_JOINT->GetForceTorque(0);
    }
    else if(PongBot.Type == WHEELMODE){
        wrench = this->RL_WHEEL_JOINT->GetForceTorque(0);
    } 
    
    #if GAZEBO_MAJOR_VERSION >= 8
        force = wrench.body2Force;
        torque = wrench.body2Torque;
    #else
        force = wrench.body2Force.Ign();
        torque = wrench.body2Torque.Ign();
    #endif

    PongBot.RL_Foot.FTSensor(PongBot.Fx) = force.X();
    PongBot.RL_Foot.FTSensor(PongBot.Fy) = force.Y();
    PongBot.RL_Foot.FTSensor(PongBot.Fz) = force.Z();
    
    
    if(PongBot.Type == LEGMODE){
        wrench = this->RR_TIP_JOINT->GetForceTorque(0);
    }
    else if(PongBot.Type == WHEELMODE){
        wrench = this->RR_WHEEL_JOINT->GetForceTorque(0);
    } 
    
    #if GAZEBO_MAJOR_VERSION >= 8
        force = wrench.body2Force;
        torque = wrench.body2Torque;
    #else
        force = wrench.body2Force.Ign();
        torque = wrench.body2Torque.Ign();
    #endif    
    
    PongBot.RR_Foot.FTSensor(PongBot.Fx) = force.X();
    PongBot.RR_Foot.FTSensor(PongBot.Fy) = force.Y();
    PongBot.RR_Foot.FTSensor(PongBot.Fz) = force.Z();

    
    if (PongBot.FL_Foot.FTSensor(PongBot.Fz) > ForceThreshold) {
        if (FL_Fz_ReadCount < ReadCountThreshold) {
            FL_Fz_ReadCount++;
        }
        else {
//            PongBot.contact.Foot(FL) = true;
        }
    }
    else {
//        PongBot.contact.Foot(FL) = false;
        FL_Fz_ReadCount = 0;
    }
    
    if (PongBot.FR_Foot.FTSensor(PongBot.Fz) > ForceThreshold) {
        if (FR_Fz_ReadCount < ReadCountThreshold) {
            FR_Fz_ReadCount++;
        }
        else {
//            PongBot.contact.Foot(FR) = true;
        }
    }
    else {
//        PongBot.contact.Foot(FR) = false;
        FR_Fz_ReadCount = 0;
    }
    
    if (PongBot.RL_Foot.FTSensor(PongBot.Fz) > ForceThreshold) {
        if (RL_Fz_ReadCount < ReadCountThreshold) {
            RL_Fz_ReadCount++;
        }
        else{
//            PongBot.contact.Foot(RL) = true;
        }
    }
    else{
//        PongBot.contact.Foot(RL) = false;
        RL_Fz_ReadCount = 0;
    }

    if (PongBot.RR_Foot.FTSensor(PongBot.Fz) > ForceThreshold) {
        if (RR_Fz_ReadCount < ReadCountThreshold) {
            RR_Fz_ReadCount++;
        }
        else {
//            PongBot.contact.Foot(RR) = true;
        }
    }
    else {
//        PongBot.contact.Foot(RR) = false;
        RR_Fz_ReadCount = 0;
    }   
    
}         

void gazebo::PongBot_plugin::GetGroundTruth() {    
    /* GetGroundTruth
     * Read GroundTruthData
     */
    #if GAZEBO_MAJOR_VERSION >= 8
        ModelPose                                   = this->model->WorldPose();
        BodyPose                                    = this->BODY->WorldPose().Pos();
        BodyVel                                     = this->BODY->WorldLinearVel();
        BodyOri                                     = this->BODY->WorldPose().Rot().Euler();
        BodyQuat.setRPY(BodyOri.X(), BodyOri.Y(), BodyOri.Z());
        
        if(PongBot.Type == LEGMODE){
            FL_FootPose                             = this->FL_TIP->WorldPose().Pos();
            FR_FootPose                             = this->FR_TIP->WorldPose().Pos();
            RL_FootPose                             = this->RL_TIP->WorldPose().Pos();
            RR_FootPose                             = this->RR_TIP->WorldPose().Pos();
        }
        
        PongBot.base.G_GroundTruthPos(AXIS_X)         = BodyPose.X();
        PongBot.base.G_GroundTruthPos(AXIS_Y)         = BodyPose.Y();
        PongBot.base.G_GroundTruthPos(AXIS_Z)         = BodyPose.Z();

        PongBot.base.G_GroundTruthVel(AXIS_X)         = BodyVel.X();
        PongBot.base.G_GroundTruthVel(AXIS_Y)         = BodyVel.Y();
        PongBot.base.G_GroundTruthVel(AXIS_Z)         = BodyVel.Z();
        
        PongBot.base.G_GroundTruthOri(AXIS_ROLL)      = BodyOri.X();
        PongBot.base.G_GroundTruthOri(AXIS_PITCH)     = BodyOri.Y();
        PongBot.base.G_GroundTruthOri(AXIS_YAW)       = BodyOri.Z();

        PongBot.base.G_GroundTruthQuat(AXIS_QUAT_X)   = BodyQuat.x();
        PongBot.base.G_GroundTruthQuat(AXIS_QUAT_Y)   = BodyQuat.y();
        PongBot.base.G_GroundTruthQuat(AXIS_QUAT_Z)   = BodyQuat.z();
        PongBot.base.G_GroundTruthQuat(AXIS_QUAT_W)   = BodyQuat.w();
        
        PongBot.FL_Foot.G_GroundTruthPos(AXIS_X)      = FL_FootPose.X();
        PongBot.FL_Foot.G_GroundTruthPos(AXIS_Y)      = FL_FootPose.Y();
        PongBot.FL_Foot.G_GroundTruthPos(AXIS_Z)      = FL_FootPose.Z();
        
        PongBot.FR_Foot.G_GroundTruthPos(AXIS_X)      = FR_FootPose.X();
        PongBot.FR_Foot.G_GroundTruthPos(AXIS_Y)      = FR_FootPose.Y();
        PongBot.FR_Foot.G_GroundTruthPos(AXIS_Z)      = FR_FootPose.Z();
        
        PongBot.RL_Foot.G_GroundTruthPos(AXIS_X)      = RL_FootPose.X();
        PongBot.RL_Foot.G_GroundTruthPos(AXIS_Y)      = RL_FootPose.Y();
        PongBot.RL_Foot.G_GroundTruthPos(AXIS_Z)      = RL_FootPose.Z();
        
        PongBot.RR_Foot.G_GroundTruthPos(AXIS_X)      = RR_FootPose.X();
        PongBot.RR_Foot.G_GroundTruthPos(AXIS_Y)      = RR_FootPose.Y();
        PongBot.RR_Foot.G_GroundTruthPos(AXIS_Z)      = RR_FootPose.Z();
        
    #else
        ModelPose   = this->model->GetWorldPose();
        BodyPose    = this->BODY->GetWorldPose().pos;
        BodyVel     = this->BODY->GetWorldLinearVel();
        BodyOri     = this->BODY->GetWorldPose().rot().euler();
        BodyQuat    = this->BODY->GetWorldPose().rot();
    #endif
}

void gazebo::PongBot_plugin::DataSave() {    
    /* DataSave
     * Save Robot Data
     */
    if(PrintDataIndex < SAVEDATA_TIME - 1)
    {        
        if(PrintDataIndex == 1){
            cout << "================== DATA SAVE START ==================" << endl;
        }
        
        PrintData[PrintDataIndex][0] = time;
        
        for(int i=1; i < SAVEDATA_LENGTH; ++i){
            PrintData[PrintDataIndex][i] = PongBot.print.Data[i];
        }

        PrintDataIndex++;
        if(PrintDataIndex == SAVEDATA_TIME)
        {
            FileSaveStopFlag    = true;
        }
    }
}

void gazebo::PongBot_plugin::FileSave() {
    /* FileSave
     * Create File(Robot Data)
     */
    if (FileSaveStopFlag == true && FileSaveDoneFlag == false)
    {
        unsigned int i, j;
        //PongBot_data = fopen("/home/ysj/Desktop/PongBot_Data.txt", "w");
        PongBot_data = fopen("/home/wonsukji/Desktop/PongBot_Data.txt", "w");

        for(i=0; i<PrintDataIndex; i++){
            for(j=0; j<SAVEDATA_LENGTH; j++){
                fprintf(PongBot_data, "%.8lf\t", PrintData[i][j]);
            }
                fprintf(PongBot_data, "\n");
        }
        fclose(PongBot_data);
        FileSaveFlag = false;        
        FileSaveStopFlag = false;
        FileSaveDoneFlag = true;
        PrintDataIndex = 0;
        
        cout << "================== DATA SAVE COMPLETE ==================" << endl;
    }
}
