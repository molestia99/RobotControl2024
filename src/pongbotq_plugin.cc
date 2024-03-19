/*
 * PongBot-Q Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * Developer : Won-Suk Ji
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

// #include <fstream>  // 외부 텍스트파일을 읽기 위해 넣었음

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h>                              // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h>      // urdf model read using RBDL
#include <Eigen/Dense>                              // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"


//* Numbers of Items and time for save //
#define SAVEDATA_LENGTH     400
#define SAVEDATA_TIME       30000

//* RCLAB CRobot Header
// #include "CRobot/CRobot.h"

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

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;






//** gazebo plugin
namespace gazebo 
{
    class PongBot_plugin : public ModelPlugin 
    {
        //* TIME variable       
        common::Time LastUpdatedTime;
        common::Time CurrentTime;
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
        

        physics::JointPtr FL_HR_JOINT,      FR_HR_JOINT,        RL_HR_JOINT,       RR_HR_JOINT;
        physics::JointPtr FL_HP_JOINT,      FR_HP_JOINT,        RL_HP_JOINT,       RR_HP_JOINT;
        physics::JointPtr FL_KN_JOINT,      FR_KN_JOINT,        RL_KN_JOINT,       RR_KN_JOINT;
        physics::JointPtr FL_TIP_JOINT,     FR_TIP_JOINT,       RL_TIP_JOINT,      RR_TIP_JOINT;
        
        physics::JointWrench wrench;
        
        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU;
        
        //* Variables for FT sensors
        ignition::math::Vector3d torque;
        ignition::math::Vector3d force;
        
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
        
        

        //** Enum
        // Joint
        enum JointNum
        {            
            // Front Left Leg's Hip Roll, Hip Pitch, Knee Pitch
            FLHR = 0,
            FLHP,
            FLKP,

            // Front Right Leg's Hip Roll, Hip Pitch, Knee Pitch
            FRHR,
            FRHP,
            FRKP,

            // Rear Left Leg's Hip Roll, Hip Pitch, Knee Pitch
            RLHR,
            RLHP,
            RLKP,

            // Rear Right Leg's Hip Roll, Hip Pitch, Knee Pitch
            RRHR,
            RRHP,
            RRKP       
        };

        // AXIS
        enum AXIS_XYZ
        {
            AXIS_X = 0,
            AXIS_Y,
            AXIS_Z
        };

        enum AXIS_RPY
        {
            AXIS_ROLL = 0,
            AXIS_PITCH,
            AXIS_YAW
        };

        //** Data Structure
        //Joint
        struct Joint
        {
            // Target values
            float TargetPos = 0.;   // desired Angular Position [rad]
            float TargetVel = 0.;   // desired Angular Velocity [rad/s]
            float TargetAcc = 0.;   // desired Angular Accelation [rad/s/s]
            float TargetTorque = 0.;

            // Actual values
            float ActualPos = 0.;
            float ActualVel = 0.;
            float ActualAcc = 0.;
            float ActualTorque = 0.;

            // PD Gains
            float Kp = 1000.;
            float Kd = 5.;

        };
        
        struct Body
        {
            // Target values
            // float TargetPhi = 0.;
            // float TargetTheta = 0.;
            // float TargetPsi = 0.;

            float TargetAngularVel[3];
            float TargetLinearAcc[3];
            float TargetAngle[3];

            // Actual values
            // float ActualPhi = 0.;
            // float ActualTheta = 0.;
            // float ActualPsi = 0.;
            float ActualAngularVel[3];
            float ActualLinearAcc[3];
            float ActualAngle[3];
        };



        Joint* mJoint = nullptr;
        Body mBody;
        

        unsigned int DoF = 0;
        
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void JointController();
        void SensorSetting();
                
        void GetLinks();
        void GetJoints();
      
        void IMUSensorRead();
      
        void EncoderRead();       

      
    };
    GZ_REGISTER_MODEL_PLUGIN(PongBot_plugin);
}

void gazebo::PongBot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {    
    

    //** for using RBDL library
    Model* PONGBOT_MODEL = new Model();
    Addons::URDFReadFromFile("/home/wonsukji/.gazebo/models/PONGBOT_Q_V2.0/urdf/PONGBOT_Q_V2.0.urdf", PONGBOT_MODEL, true, false);
    
    //** Floating dynamics 로 인해, RBDL 에서 읽는 총 자유도는 6 더 크게 읽힘.
    DoF = PONGBOT_MODEL->dof_count - 6;
    mJoint = new Joint[DoF];

    std::cout << "Total DoF: " << DoF << std::endl;




    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    this->model = _model;
    
    
    GetLinks();
    GetJoints();
    SensorSetting();
    
    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_MAGENTA "RBDL API version = %d\n" C_RESET, version_test);

    #if GAZEBO_MAJOR_VERSION >= 8
        this->LastUpdatedTime = this->model->GetWorld()->SimTime();
    #else
        this->LastUpdatedTime = this->model->GetWorld()->GetSimTime();
    #endif
    this->UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PongBot_plugin::UpdateAlgorithm, this));
    
}



void gazebo::PongBot_plugin::UpdateAlgorithm() {    
    // static int con_count = 0;
    
    #if GAZEBO_MAJOR_VERSION >= 8
        CurrentTime = this->model->GetWorld()->SimTime();
    #else
        CurrentTime = this->model->GetWorld()->GetSimTime();
    #endif
    dt = CurrentTime.Double() - this->LastUpdatedTime.Double();    
    time = time + dt;

    //* setting for getting dt at next step
    this->LastUpdatedTime = CurrentTime;
        
    EncoderRead();
    IMUSensorRead();
    // std::cout << "time in UpdateAlgorithm: " << time << std::endl;
    // std::cout << "  dt in UpdateAlgorithm: " << dt << std::endl;
        
    //* Real or simulated real-time thread time setting
    // for(size_t i = 0; i < DoF; ++i){
    //     std::cout << "*********************************************" << std::endl;
    //     std::cout << i << "th Target Pos: " << mJoint[i].TargetPos << std::endl;
    //     std::cout << i << "th Actual Pos: " << mJoint[i].ActualPos << std::endl;        
    //     std::cout << i << "th Target Vel: " << mJoint[i].TargetVel << std::endl;
    //     std::cout << i << "th Actual Vel: " << mJoint[i].ActualVel << std::endl;
    // }
    

    JointController();
    // con_count++;
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
    
    
    this->FL_TIP    = this->model->GetLink("FL_TIP");
    this->FR_TIP    = this->model->GetLink("FR_TIP");
    this->RL_TIP    = this->model->GetLink("RL_TIP");
    this->RR_TIP    = this->model->GetLink("RR_TIP");
 
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

    // if(PongBot.Type == LEGMODE){
        this->FL_TIP_JOINT  = this->model->GetJoint("FL_TIP_JOINT");
        this->FR_TIP_JOINT  = this->model->GetJoint("FR_TIP_JOINT");
        this->RL_TIP_JOINT  = this->model->GetJoint("RL_TIP_JOINT");
        this->RR_TIP_JOINT  = this->model->GetJoint("RR_TIP_JOINT");
  
}

// void gazebo::PongBot_plugin::InitROSSetting() {  
//     /* InitROSSetting
//      * Initialize ROS Setting
//      */
// //     S_JoyStick = node.subscribe("/joy", 1, &gazebo::PongBot_plugin::ROSJoyMode, this);
// //     P_PrintData = node.advertise<std_msgs::Float64MultiArray>("/pongbot_data/", 1000);    
// //     m_PrintData.data.resize(PongBot.print.DataLength);
// }


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

    //** Read Position 
    mJoint[FLHR].ActualPos = this->FL_HR_JOINT->Position(1);
    mJoint[FLHP].ActualPos = this->FL_HP_JOINT->Position(1);
    mJoint[FLKP].ActualPos = this->FL_KN_JOINT->Position(1);

    mJoint[FRHR].ActualPos = this->FR_HR_JOINT->Position(1);
    mJoint[FRHP].ActualPos = this->FR_HP_JOINT->Position(1);
    mJoint[FRKP].ActualPos = this->FR_KN_JOINT->Position(1);

    mJoint[RLHR].ActualPos = this->RL_HR_JOINT->Position(1);
    mJoint[RLHP].ActualPos = this->RL_HP_JOINT->Position(1);
    mJoint[RLKP].ActualPos = this->RL_KN_JOINT->Position(1);

    mJoint[RRHR].ActualPos = this->RR_HR_JOINT->Position(1);
    mJoint[RRHP].ActualPos = this->RR_HP_JOINT->Position(1);
    mJoint[RRKP].ActualPos = this->RR_KN_JOINT->Position(1);


    //** Read Velocity
    mJoint[FLHR].ActualVel = this->FL_HR_JOINT->GetVelocity(0);
    mJoint[FLHP].ActualVel = this->FL_HP_JOINT->GetVelocity(0);
    mJoint[FLKP].ActualVel = this->FL_KN_JOINT->GetVelocity(0);

    mJoint[FRHR].ActualVel = this->FR_HR_JOINT->GetVelocity(0);
    mJoint[FRHP].ActualVel = this->FR_HP_JOINT->GetVelocity(0);
    mJoint[FRKP].ActualVel = this->FR_KN_JOINT->GetVelocity(0);

    mJoint[RLHR].ActualVel = this->RL_HR_JOINT->GetVelocity(0);
    mJoint[RLHP].ActualVel = this->RL_HP_JOINT->GetVelocity(0);
    mJoint[RLKP].ActualVel = this->RL_KN_JOINT->GetVelocity(0);

    mJoint[RRHR].ActualVel = this->RR_HR_JOINT->GetVelocity(0);
    mJoint[RRHP].ActualVel = this->RR_HP_JOINT->GetVelocity(0);
    mJoint[RRKP].ActualVel = this->RR_KN_JOINT->GetVelocity(0);

    // for(size_t i = 0; i < DoF; ++i){
    //     std::cout << "mJoint[" << i << "]: " << mJoint[i].ActualPos << std::endl;
    // }

}

void gazebo::PongBot_plugin::JointController(void) {

    //** Simple PD Controller
    for(size_t i = 0; i < DoF; ++i){
        mJoint[i].TargetTorque = mJoint[i].Kp*(mJoint[i].TargetPos - mJoint[i].ActualPos) + mJoint[i].Kd*(mJoint[i].TargetVel - mJoint[i].ActualVel);
        // std::cout << i << "th joint torque: " << mJoint[i].TargetTorque << std::endl;
    }

//     /* JointController
//      * Control Joint Torque
//      */
    this->FL_HR_JOINT->SetForce(0, mJoint[0].TargetTorque);
    this->FL_HP_JOINT->SetForce(0, mJoint[1].TargetTorque);
    this->FL_KN_JOINT->SetForce(0, mJoint[2].TargetTorque);
    
    this->FR_HR_JOINT->SetForce(0, mJoint[3].TargetTorque);
    this->FR_HP_JOINT->SetForce(0, mJoint[4].TargetTorque);
    this->FR_KN_JOINT->SetForce(0, mJoint[5].TargetTorque);
    
    this->RL_HR_JOINT->SetForce(0, mJoint[6].TargetTorque);
    this->RL_HP_JOINT->SetForce(0, mJoint[7].TargetTorque);
    this->RL_KN_JOINT->SetForce(0, mJoint[8].TargetTorque);

    this->RR_HR_JOINT->SetForce(0, mJoint[9].TargetTorque);
    this->RR_HP_JOINT->SetForce(0, mJoint[10].TargetTorque);
    this->RR_KN_JOINT->SetForce(0, mJoint[11].TargetTorque);  
}

void gazebo::PongBot_plugin::IMUSensorRead() {
    // IMU sensor data    
    // static double IMU_CurrentAngle[3], IMU_CurrentAngularVel[3], IMU_CurrentLinearAcc[3];

    
    #if GAZEBO_MAJOR_VERSION >= 8
        mBody.ActualAngularVel[AXIS_ROLL]    = static_cast<float>(this->IMU->AngularVelocity(false)[AXIS_ROLL]);
        mBody.ActualAngularVel[AXIS_PITCH]   = static_cast<float>(this->IMU->AngularVelocity(false)[AXIS_PITCH]);
        mBody.ActualAngularVel[AXIS_YAW]     = static_cast<float>(this->IMU->AngularVelocity(false)[AXIS_YAW]);
        
        mBody.ActualLinearAcc[AXIS_X]        = static_cast<float>(this->IMU->LinearAcceleration()[AXIS_X]);
        mBody.ActualLinearAcc[AXIS_Y]        = static_cast<float>(this->IMU->LinearAcceleration()[AXIS_Y]);
        mBody.ActualLinearAcc[AXIS_Z]        = static_cast<float>(this->IMU->LinearAcceleration()[AXIS_Z]);
        
        mBody.ActualAngle[AXIS_ROLL]         = static_cast<float>(this->IMU->Orientation().Euler()[AXIS_ROLL]);
        mBody.ActualAngle[AXIS_PITCH]        = static_cast<float>(this->IMU->Orientation().Euler()[AXIS_PITCH]);
        mBody.ActualAngle[AXIS_YAW]          = static_cast<float>(this->IMU->Orientation().Euler()[AXIS_YAW]);
        
    #else
        // mBody.ActualAngularVel[AXIS_ROLL]    = this->IMU->AngularVelocity(false)[AXIS_ROLL];
        // mBody.ActualAngularVel[AXIS_PITCH]   = this->IMU->AngularVelocity(false)[AXIS_PITCH];
        // mBody.ActualAngularVel[AXIS_YAW]     = this->IMU->AngularVelocity(false)[AXIS_YAW];

        // mBody.ActualAngle[AXIS_ROLL]         = this->IMU->Orientation().Euler()[AXIS_ROLL];
        // mBody.ActualAngle[AXIS_PITCH]        = this->IMU->Orientation().Euler()[AXIS_PITCH];
        // mBody.ActualAngle[AXIS_YAW]          = this->IMU->Orientation().Euler()[AXIS_YAW];
    #endif

    std::cout << "!!" << std::endl;
    for(size_t i = 0; i < 3; ++i){
        std::cout << mBody.ActualLinearAcc[i] << std::endl;
    }
        
    // PongBot.imu.AngularVel[AXIS_ROLL]       = IMU_CurrentAngularVel[AXIS_ROLL];
    // PongBot.imu.AngularVel[AXIS_PITCH]      = IMU_CurrentAngularVel[AXIS_PITCH];
    // PongBot.imu.AngularVel[AXIS_YAW]        = IMU_CurrentAngularVel[AXIS_YAW];

    // PongBot.imu.LinearAcc[AXIS_X]           = IMU_CurrentLinearAcc[AXIS_X];
    // PongBot.imu.LinearAcc[AXIS_Y]           = IMU_CurrentLinearAcc[AXIS_Y];
    // PongBot.imu.LinearAcc[AXIS_Z]           = IMU_CurrentLinearAcc[AXIS_Z];
    
    // PongBot.imu.TempOri_ZYX(AXIS_ROLL)      = IMU_CurrentAngle[AXIS_ROLL];
    // PongBot.imu.TempOri_ZYX(AXIS_PITCH)     = IMU_CurrentAngle[AXIS_PITCH];
    // PongBot.imu.TempOri_ZYX(AXIS_YAW)       = IMU_CurrentAngle[AXIS_YAW];            
}

