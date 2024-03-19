
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <vector>
#include "CRobot.h"
#include "nrutil/nrlin.h"

#include <chrono>

CRobot::CRobot() {
}

CRobot::CRobot(const CRobot& orig) {
}

CRobot::~CRobot() {

}

void CRobot::initializeCRobot(Model* getModel) {
   /* initializeCRobot
    * function : initializeSystemID()
    * Initialize joint Position & Set Link Offset
    * RobotState includes [Base Position(3), Base Orientation-Quaternion(4), Joint(12)]
    * RobotStatedot includes [Base Velocity(3), Base AngularVelocity(3), JointVel(12)]
    */
    m_pModel                        = getModel;
    m_pModel->gravity               = Vector3d(0., 0., -GRAVITY);
    joint_DoF                       = m_pModel->dof_count - 6;     //* get Degree of freedom, Except x,y,z,roll,pitch,yaw of the robot
    joint                           = new JOINT[joint_DoF];        //* only joint of the robot excepting x,y,z,roll,pitch,yaw of the robot

    for (int j = 0; j < joint_DoF; j++) {
        joint[j] = {0};
    }

    //* Initialize VectorNd RobotState, RobotStatedot, RobotState2dot
    RobotState                          = VectorNd::Zero(m_pModel->dof_count + 1);
    RobotStatedot                       = VectorNd::Zero(m_pModel->dof_count);
    RobotState2dot                      = VectorNd::Zero(m_pModel->dof_count);    
    
    RobotState_local                    = VectorNd::Zero(m_pModel->dof_count + 1);
    
    base.ID                             = m_pModel->GetBodyId("BODY");
    
    if(Type == LEGMODE){
        FL_Foot.ID                      = m_pModel->GetBodyId("FL_TIP");
        FR_Foot.ID                      = m_pModel->GetBodyId("FR_TIP");
        RL_Foot.ID                      = m_pModel->GetBodyId("RL_TIP");
        RR_Foot.ID                      = m_pModel->GetBodyId("RR_TIP");
    }
    else if(Type == WHEELMODE){
        FL_Foot.ID                      = m_pModel->GetBodyId("FL_WHEEL");
        FR_Foot.ID                      = m_pModel->GetBodyId("FR_WHEEL");
        RL_Foot.ID                      = m_pModel->GetBodyId("RL_WHEEL");
        RR_Foot.ID                      = m_pModel->GetBodyId("RR_WHEEL");        
    }
    
    J_RL                                = MatrixNd::Zero(3, joint_DoF+6);
    J_RR                                = MatrixNd::Zero(3, joint_DoF+6);
    J_FL                                = MatrixNd::Zero(3, joint_DoF+6);
    J_FR                                = MatrixNd::Zero(3, joint_DoF+6);

    J_BASE                              = MatrixNd::Zero(6, joint_DoF+6);
//    J_A                                 = MatrixNd::Zero(joint_DoF+6, joint_DoF+6);
    
    //* print(DoFs of Robot and DoFs of Joints)    
    printf(C_BLACK "The Robot's DoFs                      =  %d\n" C_RESET, m_pModel->dof_count);
    printf(C_BLACK "The Robot's Joint DoFs                =  %d\n" C_RESET, joint_DoF);
    
    wheel.SpeedWeight               = 15;
}

void CRobot::setPrintData(int DataLength, int SaveTime) {
   /* setPrintData
    * Set SAVEDATA Length, Time[ms]
    */
    
    print.DataLength                = DataLength;
    print.SaveTime                  = SaveTime;
    print.Data                      = VectorXf::Zero(DataLength);
}

void CRobot::setMode(bool ModeSelection, int TypeSelection) {
   /* setMode
    * Set Mode (Simulation or Actual)
    */
    if (ModeSelection == SIMULATION){
        cout << "PONGBOT MODE : SIMULATION" << endl;
        Mode        = SIMULATION;
        
        tasktime    = 0.002;
        onesecSize  = 1.0/tasktime;
    }
    else if (ModeSelection == ACTUAL_ROBOT){
        cout << "PONGBOT MODE : ACTUAL" << endl;
        Mode = ACTUAL_ROBOT;
        
        tasktime    = 0.002;
        onesecSize  = 1.0/tasktime;
    }

    if (TypeSelection == LEGMODE){        
        Type = LEGMODE;
        cout << "PONGBOT TYPE : LEG" << endl;
    }
    else if (TypeSelection == WHEELMODE){
        Type = WHEELMODE;
        cout << "PONGBOT TYPE : WHEEL" << endl;
    }   
}

void CRobot::setRobotModel(Model* getModel) {
   /* setRobotModel
    * input : urdf model data,
    * output : DOF of robot, create joint space of legs
    */
    initializeCRobot(getModel);
    initializeSystemID();
    initializeParameter();
    initializeHardWare();
    initializeOSQP();
    ControlMode = CTRLMODE_NONE;
    CommandFlag = NONE_ACT;
}

void CRobot::initializeHardWare() {
   /* HardWareSetting
    * 
    */
        hardware.GearRatio                  <<        50,             50,             50,
                                                      50,             50,             50,
                                                      50,             50,             50,
                                                      50,             50,             50;

    if (Mode == ACTUAL_ROBOT){
        hardware.TorqueSensitivity          <<      2.85,           8.57,            8.9,
                                                    2.85,           8.57,            8.9,
                                                    2.85,           8.57,            8.9,
                                                    2.85,           8.57,            8.9,
        
        hardware.RatedCurrent               <<      0.159,          0.125,          0.210,
                                                    0.159,          0.125,          0.210,
                                                    0.159,          0.125,          0.210,
                                                    0.159,          0.125,          0.210;
        
        hardware.ABS_EncoderResolution      <<      262144,         262144,         16384,
                                                    262144,         262144,         16384,
                                                    262144,         262144,         16384,
                                                    262144,         262144,         16384;


        friction.Coulomb		    <<      0.800000015895570,     2.999999902932275,     1.871862549481192,
                                                    0.800000135856877,     1.004423607864183,     1.701846151110656,
                                                    0.800000015895570,     0.800020553831293,     1.967365269816205,
                                                    0.800000135856877,     0.800000173687293,     1.906943044940459;

        friction.Viscous        	    <<      0.699999788611817,     2.299999936733397,     2.099999963188763,
                                                    0.699999733494695,     1.599999979977726,     1.999999951790426,
                                                    0.699999788611817,     1.099999997007465,     2.599999949920193,
                                                    0.699999733494695,     1.299999961627990,     2.599999948671154;

        // friction.Inertia        	    <<      2.667200584629277e-08,         0.407977492090722,     0.291248836385921,
        //                                      8.874356043737144e-08,     1.186033795706419e-08,     0.374147749035321,
        //                                      2.667200584629277e-08,     4.326335602228652e-09,     0.256553236866393,
        //                                      8.874356043737144e-08,     1.477380943234562e-08,     0.257057039226771;

        friction.Activeflag 		    = true;
    }
}

void CRobot::initializeSystemID(){
    /* initializeSystemID
     * input  : mass, com Height of robot
     * output : Setting robot's com height, mass, moment of Inertia
     */
    
    //* Calculate kinematics data using URDF file
    int link_id;
    Vector3f offset;
    
    //Body
    link_id                                 = m_pModel->GetBodyId("BODY");
    BodyKine.i_body_link                    = link_id;
    BodyKine.m_body_link                    = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_body_link                    = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    // Hip Link
    link_id                                 = m_pModel->GetBodyId("FL_HIP");
    BodyKine.i_FL_hip_link                  = link_id;
    BodyKine.m_FL_hip_link                  = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_FL_hip_link                  = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("FR_HIP");
    BodyKine.i_FR_hip_link                  = link_id;
    BodyKine.m_FR_hip_link                  = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_FR_hip_link                  = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RL_HIP");
    BodyKine.i_RL_hip_link                  = link_id;
    BodyKine.m_RL_hip_link                  = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_RL_hip_link                  = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RR_HIP");
    BodyKine.i_RR_hip_link                  = link_id;
    BodyKine.m_RR_hip_link                  = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_RR_hip_link                  = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    // Thigh Link
    link_id                                 = m_pModel->GetBodyId("FL_THIGH");
    BodyKine.i_FL_thigh_link                = link_id;
    BodyKine.m_FL_thigh_link                = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_FL_thigh_link                = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("FR_THIGH");
    BodyKine.i_FR_thigh_link                = link_id;
    BodyKine.m_FR_thigh_link                = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_FR_thigh_link                = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RL_THIGH");
    BodyKine.i_RL_thigh_link                = link_id;
    BodyKine.m_RL_thigh_link                = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_RL_thigh_link                = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RR_THIGH");
    BodyKine.i_RR_thigh_link                = link_id;
    BodyKine.m_RR_thigh_link                = m_pModel->mBodies[link_id].mMass;
    BodyKine.c_RR_thigh_link                = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    
    if(Type == LEGMODE){        
        link_id                                = m_pModel->GetBodyId("FL_TIP");
        BodyKine.i_FL_tip_link                 = link_id;
        BodyKine.m_FL_tip_link                 = 0.1646;
//        BodyKine.m_FL_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_FL_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
        
        link_id                                = m_pModel->GetBodyId("FR_TIP");
        BodyKine.i_FR_tip_link                 = link_id;
        BodyKine.m_FR_tip_link                 = 0.1646;
//        BodyKine.m_FR_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_FR_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();

        link_id                                = m_pModel->GetBodyId("RL_TIP");
        BodyKine.i_RL_tip_link                 = link_id;
        BodyKine.m_RL_tip_link                 = 0.1646;
//        BodyKine.m_RL_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_RL_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
        
        link_id                                = m_pModel->GetBodyId("RR_TIP");
        BodyKine.i_RR_tip_link                 = link_id;
        BodyKine.m_RR_tip_link                 = 0.1646;
//        BodyKine.m_RR_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_RR_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();        
    }
    else if(Type == WHEELMODE){
        link_id                                = m_pModel->GetBodyId("FL_WHEEL");
        BodyKine.i_FL_tip_link                 = link_id;
        BodyKine.m_FL_tip_link                 = 0.1646; 
//        BodyKine.m_FL_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_FL_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
        
        link_id                                = m_pModel->GetBodyId("FR_WHEEL");
        BodyKine.i_FR_tip_link                 = link_id;
        BodyKine.m_FR_tip_link                 = 0.1646;
//        BodyKine.m_FR_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_FR_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
        
        link_id                                = m_pModel->GetBodyId("RL_WHEEL");
        BodyKine.i_RL_tip_link                 = link_id;
        BodyKine.m_RL_tip_link                 = 0.1646;
//        BodyKine.m_RL_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_RL_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
        
        link_id                                = m_pModel->GetBodyId("RR_WHEEL");
        BodyKine.i_RR_tip_link                 = link_id;
        BodyKine.m_RR_tip_link                 = 0.1646;
//        BodyKine.m_RR_tip_link                 = m_pModel->mBodies[link_id].mMass;
//        BodyKine.c_RR_tip_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    }
    
    // Calf Link    
    link_id                                 = m_pModel->GetBodyId("FL_CALF");
    BodyKine.i_FL_calf_link                 = link_id;
    BodyKine.m_FL_calf_link                 = m_pModel->mBodies[link_id].mMass - BodyKine.m_FL_tip_link;
    BodyKine.c_FL_calf_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("FR_CALF");
    BodyKine.i_FR_calf_link                 = link_id;
    BodyKine.m_FR_calf_link                 = m_pModel->mBodies[link_id].mMass - BodyKine.m_FR_tip_link;
    BodyKine.c_FR_calf_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RL_CALF");
    BodyKine.i_RL_calf_link                 = link_id;
    BodyKine.m_RL_calf_link                 = m_pModel->mBodies[link_id].mMass - BodyKine.m_RL_tip_link;
    BodyKine.c_RL_calf_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    link_id                                 = m_pModel->GetBodyId("RR_CALF");
    BodyKine.i_RR_calf_link                 = link_id;
    BodyKine.m_RR_calf_link                 = m_pModel->mBodies[link_id].mMass - BodyKine.m_RR_tip_link;
    BodyKine.c_RR_calf_link                 = m_pModel->mBodies[link_id].mCenterOfMass.cast <float> ();
    
    BodyKine.TargetCoMHeight              =    0.40; 
    BodyKine.BaseHeight                   =    0.436;

    BodyKine.m_total                      = (BodyKine.m_body_link)      +
                                            (BodyKine.m_FL_hip_link     + BodyKine.m_FR_hip_link    + BodyKine.m_RL_hip_link    + BodyKine.m_RR_hip_link)   + 
                                            (BodyKine.m_FL_thigh_link   + BodyKine.m_FR_thigh_link  + BodyKine.m_RL_thigh_link  + BodyKine.m_RR_thigh_link) + 
                                            (BodyKine.m_FL_calf_link    + BodyKine.m_FR_calf_link   + BodyKine.m_RL_calf_link   + BodyKine.m_RR_calf_link)  +
                                            (BodyKine.m_FL_tip_link     + BodyKine.m_FR_tip_link    + BodyKine.m_RL_tip_link    + BodyKine.m_RR_tip_link);
    
    if(Type == LEGMODE){
        BodyKine.B_I_g                                   <<    1.358181,    -0.002226,    0.104939,
                                                               0.002226,     3.840904,   -0.000665,
                                                               0.104939,    -0.000665,    4.399234;
    }
    else if(Type == WHEELMODE){
        BodyKine.B_I_g                                   <<    3.193732,    -0.011649,    0.027769,
                                                              -0.011649,     6.037623,   -0.005104,
                                                               0.027769,    -0.005104,    5.599209;
    }
    
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            BodyKine.B_I_tensor[row + 1][column + 1]  = BodyKine.B_I_g(row, column);
        }
    }
}

void CRobot::initializeParameter(){
    /* initializeParameter
     * input : 
     * output : 
     */
    float** OutMat = matrix(1, 3, 1, 3);
    
    // Initialize Body Parameter
    base.G_InitPos                              << 0, 0, BodyKine.BaseHeight;
    base.G_InitVel                              << 0, 0, 0;
    base.G_InitAcc                              << 0, 0, 0;    
    base.G_InitOri                              << 0, 0, 0;
    base.G_InitAngularVel                       << 0, 0, 0;
    base.G_InitAngularAcc                       << 0, 0, 0;

    base.G_InitQuat                             = Math::Quaternion::fromZYXAngles(((base.G_InitOri.cast <double> ()).tail(3)).reverse());
    
    Math::Quaternion BaseInitQuat(base.G_InitQuat);
    m_pModel->SetQuaternion(base.ID, BaseInitQuat, RobotState);
    m_pModel->SetQuaternion(base.ID, BaseInitQuat, RobotState_local);
    
    base.G_CurrentPos                           = base.G_InitPos;
    base.G_CurrrentVel                          = base.G_InitVel;
    base.G_CurrentAcc                           = base.G_InitAcc;
    base.G_CurrentOri                           = base.G_InitOri;   
    base.G_CurrentAngularVel                    = base.G_InitAngularVel;
    base.G_CurrentAngularAcc                    = base.G_InitAngularAcc;
    
    base.G_RefPos                               = base.G_InitPos;
    base.G_RefVel                               = base.G_InitVel;
    base.G_RefAcc                               = base.G_InitAcc;
    base.G_RefOri                               = base.G_InitOri;   
    base.G_RefAngularVel                        = base.G_InitAngularVel;
    base.G_RefAngularAcc                        = base.G_InitAngularAcc;
    
    setRotationZYX(base.G_RefOri, Ref_C_gb);
    setRotationZYX(base.G_CurrentOri, Current_C_gb);
    
    mmult(Current_C_gb, 3, 3, BodyKine.B_I_tensor, 3, 3, OutMat);
    mmult(OutMat, 3, 3, Current_C_gb, 3, 3, BodyKine.G_I_tensor);
    
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            BodyKine.G_I_g(row, column)   = BodyKine.G_I_tensor[row + 1][column + 1];
        }
    }
        
    // Initialize Leg Parameter
    lowerbody.BASE_TO_HR_LENGTH                 <<    0.31804,     0.11380,            0;
    lowerbody.HR_TO_HP_LENGTH                   <<          0,       0.105,            0;
    lowerbody.HP_TO_KN_LENGTH                   <<          0,           0,        0.305;    
    
    if(Type == LEGMODE){         
        lowerbody.KN_TO_EP_LENGTH               <<          0,           0,      0.28816;
    }         
    else if(Type == WHEELMODE){         
        lowerbody.KN_TO_EP_LENGTH               <<   0.032441,           0,      0.32759;     
    }         
             
    lowerbody.FL_BASE_TO_HR                     <<  lowerbody.BASE_TO_HR_LENGTH(AXIS_X),  lowerbody.BASE_TO_HR_LENGTH(AXIS_Y), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z);
    lowerbody.FR_BASE_TO_HR                     <<  lowerbody.BASE_TO_HR_LENGTH(AXIS_X), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Y), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z);
    lowerbody.RL_BASE_TO_HR                     << -lowerbody.BASE_TO_HR_LENGTH(AXIS_X),  lowerbody.BASE_TO_HR_LENGTH(AXIS_Y), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z);
    lowerbody.RR_BASE_TO_HR                     << -lowerbody.BASE_TO_HR_LENGTH(AXIS_X), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Y), -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z);
         
    lowerbody.FL_HR_TO_HP                       <<    lowerbody.HR_TO_HP_LENGTH(AXIS_X),    lowerbody.HR_TO_HP_LENGTH(AXIS_Y),    lowerbody.HR_TO_HP_LENGTH(AXIS_Z);
    lowerbody.FR_HR_TO_HP                       <<    lowerbody.HR_TO_HP_LENGTH(AXIS_X),   -lowerbody.HR_TO_HP_LENGTH(AXIS_Y),    lowerbody.HR_TO_HP_LENGTH(AXIS_Z);
    lowerbody.RL_HR_TO_HP                       <<    lowerbody.HR_TO_HP_LENGTH(AXIS_X),    lowerbody.HR_TO_HP_LENGTH(AXIS_Y),    lowerbody.HR_TO_HP_LENGTH(AXIS_Z);
    lowerbody.RR_HR_TO_HP                       <<    lowerbody.HR_TO_HP_LENGTH(AXIS_X),   -lowerbody.HR_TO_HP_LENGTH(AXIS_Y),    lowerbody.HR_TO_HP_LENGTH(AXIS_Z);
         
    lowerbody.FL_HP_TO_KN                       <<    lowerbody.HP_TO_KN_LENGTH(AXIS_X),    lowerbody.HP_TO_KN_LENGTH(AXIS_Y),   -lowerbody.HP_TO_KN_LENGTH(AXIS_Z);
    lowerbody.FR_HP_TO_KN                       <<    lowerbody.HP_TO_KN_LENGTH(AXIS_X),    lowerbody.HP_TO_KN_LENGTH(AXIS_Y),   -lowerbody.HP_TO_KN_LENGTH(AXIS_Z);
    lowerbody.RL_HP_TO_KN                       <<    lowerbody.HP_TO_KN_LENGTH(AXIS_X),    lowerbody.HP_TO_KN_LENGTH(AXIS_Y),   -lowerbody.HP_TO_KN_LENGTH(AXIS_Z);
    lowerbody.RR_HP_TO_KN                       <<    lowerbody.HP_TO_KN_LENGTH(AXIS_X),    lowerbody.HP_TO_KN_LENGTH(AXIS_Y),   -lowerbody.HP_TO_KN_LENGTH(AXIS_Z);
         
    if(Type == LEGMODE){         
        lowerbody.FL_KN_TO_EP                   <<    lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.FR_KN_TO_EP                   <<    lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.RL_KN_TO_EP                   <<    lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.RR_KN_TO_EP                   <<    lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
    }         
    else if(Type == WHEELMODE){         
        lowerbody.FL_KN_TO_EP                   <<   -lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.FR_KN_TO_EP                   <<   -lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.RL_KN_TO_EP                   <<   -lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
        lowerbody.RR_KN_TO_EP                   <<   -lowerbody.KN_TO_EP_LENGTH(AXIS_X),    lowerbody.KN_TO_EP_LENGTH(AXIS_Y),   -lowerbody.KN_TO_EP_LENGTH(AXIS_Z);
    }         
         
    lowerbody.FL_BASE_TO_EP                     =     lowerbody.FL_BASE_TO_HR + lowerbody.FL_HR_TO_HP + lowerbody.FL_HP_TO_KN + lowerbody.FL_KN_TO_EP;
    lowerbody.FR_BASE_TO_EP                     =     lowerbody.FR_BASE_TO_HR + lowerbody.FR_HR_TO_HP + lowerbody.FR_HP_TO_KN + lowerbody.FR_KN_TO_EP;
    lowerbody.RL_BASE_TO_EP                     =     lowerbody.RL_BASE_TO_HR + lowerbody.RL_HR_TO_HP + lowerbody.RL_HP_TO_KN + lowerbody.RL_KN_TO_EP;
    lowerbody.RR_BASE_TO_EP                     =     lowerbody.RR_BASE_TO_HR + lowerbody.RR_HR_TO_HP + lowerbody.RR_HP_TO_KN + lowerbody.RR_KN_TO_EP;


    // Initialize Foot Position (Inertial Frame)
    
    static float Foot_offset_x                  = 0.02;
    static float Foot_offset_y                  = 0.02;
    
    FL_Foot.G_InitPos                           << (lowerbody.FL_BASE_TO_HR + lowerbody.FL_HR_TO_HP)(AXIS_X) - Foot_offset_x, (lowerbody.FL_BASE_TO_HR + lowerbody.FL_HR_TO_HP)(AXIS_Y) - Foot_offset_y, 0;
    FR_Foot.G_InitPos                           << (lowerbody.FR_BASE_TO_HR + lowerbody.FR_HR_TO_HP)(AXIS_X) - Foot_offset_x, (lowerbody.FR_BASE_TO_HR + lowerbody.FR_HR_TO_HP)(AXIS_Y) + Foot_offset_y, 0;
    RL_Foot.G_InitPos                           << (lowerbody.RL_BASE_TO_HR + lowerbody.RL_HR_TO_HP)(AXIS_X) + Foot_offset_x, (lowerbody.RL_BASE_TO_HR + lowerbody.RL_HR_TO_HP)(AXIS_Y) - Foot_offset_y, 0;
    RR_Foot.G_InitPos                           << (lowerbody.RR_BASE_TO_HR + lowerbody.RR_HR_TO_HP)(AXIS_X) + Foot_offset_x, (lowerbody.RR_BASE_TO_HR + lowerbody.RR_HR_TO_HP)(AXIS_Y) + Foot_offset_y, 0;

    FL_Foot.G_RefPos                            = FL_Foot.G_InitPos;
    FR_Foot.G_RefPos                            = FR_Foot.G_InitPos;
    RL_Foot.G_RefPos                            = RL_Foot.G_InitPos;
    RR_Foot.G_RefPos                            = RR_Foot.G_InitPos;
    
    FL_Foot.G_CurrentPos                        = FL_Foot.G_InitPos;
    FR_Foot.G_CurrentPos                        = FR_Foot.G_InitPos;
    RL_Foot.G_CurrentPos                        = RL_Foot.G_InitPos;
    RR_Foot.G_CurrentPos                        = RR_Foot.G_InitPos;

    FL_Foot.G_RefTargetPos                      = FL_Foot.G_InitPos;
    FR_Foot.G_RefTargetPos                      = FR_Foot.G_InitPos;
    RL_Foot.G_RefTargetPos                      = RL_Foot.G_InitPos;
    RR_Foot.G_RefTargetPos                      = RR_Foot.G_InitPos;
        
    FL_Foot.B_RefPos                            = FL_Foot.G_InitPos - base.G_InitPos;
    FR_Foot.B_RefPos                            = FR_Foot.G_InitPos - base.G_InitPos;    
    RL_Foot.B_RefPos                            = RL_Foot.G_InitPos - base.G_InitPos;
    RR_Foot.B_RefPos                            = RR_Foot.G_InitPos - base.G_InitPos;
    
    FL_Foot.B_CurrentPos                        = FL_Foot.B_RefPos;
    FR_Foot.B_CurrentPos                        = FR_Foot.B_RefPos;
    RL_Foot.B_CurrentPos                        = RL_Foot.B_RefPos;
    RR_Foot.B_CurrentPos                        = RR_Foot.B_RefPos;
    
    // Initialize Joint Position
    
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint){
        joint[nJoint].RefPos                    = 0;
        joint[nJoint].RefPrePos                 = 0;
        joint[nJoint].RefVel                    = 0;
        joint[nJoint].RefAcc                    = 0;

        joint[nJoint].CurrentPos                = 0;
        joint[nJoint].CurrentVel                = 0;
        joint[nJoint].CurrentPreVel             = 0;
        joint[nJoint].CurrentAcc                = 0;

        joint[nJoint].RefTargetPos              = 0;
        joint[nJoint].RefTargetVel              = 0;
        joint[nJoint].RefTargetAcc              = 0;

        joint[nJoint].RefTorque                 = 0;
    }
    VectorXf EP_InitPos(12);
    VectorXf Joint_RefAngle(12);
    
    // Inverse Kinematics    
    EP_InitPos                                  << FL_Foot.B_RefPos, FR_Foot.B_RefPos, RL_Foot.B_RefPos, RR_Foot.B_RefPos;
    Joint_RefAngle                              = computeIK(EP_InitPos);
    
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint){
        joint[nJoint].RefTargetPos              = Joint_RefAngle[nJoint];
        joint[nJoint].RefPos                    = Joint_RefAngle[nJoint];
        joint[nJoint].RefPrePos                 = Joint_RefAngle[nJoint];
    }
    
    // Initialize CoM Parameter
    com.G_InitPos                               = GetCOM(base.G_InitPos, base.G_InitOri, Joint_RefAngle);
    com.G_InitVel                               << 0, 0, 0;
    com.G_InitAcc                               << 0, 0, 0;
        
    com.G_RefPos                                = com.G_InitPos;
    com.G_RefVel                                = com.G_InitVel;
    com.G_RefAcc                                = com.G_InitAcc;
    
    com.G_CurrentPos                            = com.G_InitPos;
    com.G_CurrrentVel                           = com.G_InitVel;
    com.G_CurrentAcc                            = com.G_InitAcc;
   
    // CoM G_RefTargetPosition
    com.G_RefTargetPos                          << 0, 0, BodyKine.TargetCoMHeight;
    com.G_RefTargetVel                          = com.G_RefVel; 
    com.G_RefTargetAcc                          = com.G_RefAcc; 
    
    
    // Base - CoM Offset (Position)
    com.G_Base_to_CoM_RefPos                    = com.G_InitPos - base.G_InitPos;
    com.B_Base_to_CoM_RefPos                    = com.G_Base_to_CoM_RefPos;
    com.G_Base_to_CoM_CurrentPos                = com.G_Base_to_CoM_RefPos;
    com.B_Base_to_CoM_CurrentPos                = com.G_Base_to_CoM_RefPos;
    
    com.G_CoM_to_Base_RefPos                    = base.G_InitPos - com.G_InitPos;
    com.B_CoM_to_Base_RefPos                    = com.G_CoM_to_Base_RefPos;
    com.G_CoM_to_Base_CurrentPos                = com.G_CoM_to_Base_RefPos;
    com.B_CoM_to_Base_CurrentPos                = com.G_CoM_to_Base_RefPos;
    
    //For Calculation(matrix)
    B_Base2CoM_RefPos[AXIS_X + 1]               = com.B_Base_to_CoM_RefPos(AXIS_X);
    B_Base2CoM_RefPos[AXIS_Y + 1]               = com.B_Base_to_CoM_RefPos(AXIS_Y);
    B_Base2CoM_RefPos[AXIS_Z + 1]               = com.B_Base_to_CoM_RefPos(AXIS_Z);
    
    B_CoM2Base_RefPos[AXIS_X + 1]               = com.B_CoM_to_Base_RefPos(AXIS_X);
    B_CoM2Base_RefPos[AXIS_Y + 1]               = com.B_CoM_to_Base_RefPos(AXIS_Y);
    B_CoM2Base_RefPos[AXIS_Z + 1]               = com.B_CoM_to_Base_RefPos(AXIS_Z);
    
    
    // Base - CoM Offset (Velocity)
    com.G_Base_to_CoM_RefVel                    = com.G_InitVel - base.G_InitVel;
    com.B_Base_to_CoM_RefVel                    = com.G_Base_to_CoM_RefVel;
    com.G_Base_to_CoM_CurrentVel                = com.G_Base_to_CoM_RefVel;
    com.B_Base_to_CoM_CurrentVel                = com.G_Base_to_CoM_RefVel;
    
    com.G_CoM_to_Base_RefVel                    = base.G_InitVel - com.G_InitVel;
    com.B_CoM_to_Base_RefVel                    = com.G_CoM_to_Base_RefVel;
    com.G_CoM_to_Base_CurrentVel                = com.G_CoM_to_Base_RefVel;
    com.B_CoM_to_Base_CurrentVel                = com.G_CoM_to_Base_RefVel;
    

    // Initialize CoM->Foot Position 
    FL_Foot.G_CoM_to_Foot_RefPos                = FL_Foot.G_InitPos - com.G_InitPos;
    FR_Foot.G_CoM_to_Foot_RefPos                = FR_Foot.G_InitPos - com.G_InitPos;
    RL_Foot.G_CoM_to_Foot_RefPos                = RL_Foot.G_InitPos - com.G_InitPos;
    RR_Foot.G_CoM_to_Foot_RefPos                = RR_Foot.G_InitPos - com.G_InitPos;

    FL_Foot.G_CoM_to_Foot_CurrentPos            = FL_Foot.G_CoM_to_Foot_RefPos;
    FR_Foot.G_CoM_to_Foot_CurrentPos            = FR_Foot.G_CoM_to_Foot_RefPos;    
    RL_Foot.G_CoM_to_Foot_CurrentPos            = RL_Foot.G_CoM_to_Foot_RefPos;
    RR_Foot.G_CoM_to_Foot_CurrentPos            = RR_Foot.G_CoM_to_Foot_RefPos;
    
    FL_Foot.B_CoM_to_Foot_RefPos                = FL_Foot.G_CoM_to_Foot_RefPos;
    FR_Foot.B_CoM_to_Foot_RefPos                = FR_Foot.G_CoM_to_Foot_RefPos;
    RL_Foot.B_CoM_to_Foot_RefPos                = RL_Foot.G_CoM_to_Foot_RefPos;
    RR_Foot.B_CoM_to_Foot_RefPos                = RR_Foot.G_CoM_to_Foot_RefPos;
    
    FL_Foot.B_CoM_to_Foot_CurrentPos            = FL_Foot.G_CoM_to_Foot_RefPos;
    FR_Foot.B_CoM_to_Foot_CurrentPos            = FR_Foot.G_CoM_to_Foot_RefPos;
    RL_Foot.B_CoM_to_Foot_CurrentPos            = RL_Foot.G_CoM_to_Foot_RefPos;
    RR_Foot.B_CoM_to_Foot_CurrentPos            = RR_Foot.G_CoM_to_Foot_RefPos;
    
    // Initialize Foot->CoM Position 
    com.G_Foot_to_CoM_RefPos                    = com.G_InitPos;
    com.G_Foot_to_CoM_CurrentPos                = com.G_InitPos;
            
    com.G_Foot_to_CoM_RefVel                    = com.G_InitVel;
    com.G_Foot_to_CoM_CurrentVel                = com.G_InitVel;
    
    osqp.Select.block<12, 12>(0, 6)             = MatrixXf::Identity(12, 12);

    // Initialize Gain   
    gain.JointKp                                << 450, 450, 2800,
                                                   450, 450, 2800,
                                                   450, 450, 2800,
                                                   450, 450, 2800;
        
    if(Mode == ACTUAL_ROBOT){
        gain.JointKd                            <<  50,  50,  300,
                                                    50,  50,  300,
                                                    50,  50,  300,
                                                    50,  50,  300;
    }
    else if(Mode == SIMULATION){        
        gain.JointKd                            <<  50,  50,  300,
                                                    50,  50,  300,
                                                    50,  50,  300,
                                                    50,  50,  300;
    }
    
    gain.BasePosKp                              <<  0, 0, 0,
                                                    0, 0, 0;
    
    gain.BasePosKd                              <<  0, 0, 0,
                                                    0, 0, 0;
        
    gain.TaskKp                                 <<  0, 0, 0,
                                                    0, 0, 0,
                                                    0, 0, 0,
                                                    0, 0, 0;
    
    gain.TaskKd                                 <<  0, 0, 0,
                                                    0, 0, 0,
                                                    0, 0, 0,
                                                    0, 0, 0;

    if(Mode == ACTUAL_ROBOT){
        walking.trot.SpeedScale(AXIS_X)         = 0.6;
        walking.trot.SpeedScale(AXIS_Y)         = 0.3;
        walking.trot.SpeedScale(AXIS_YAW)       = 0.5;
        
        walking.trot.LimitAcceleration(AXIS_X)  = 0.5;
        walking.trot.LimitAcceleration(AXIS_Y)  = 0.5;
        walking.trot.LimitAcceleration(AXIS_YAW)= 0.5;
        
        walking.trot.ActiveThreshold            = 0.1;
        
        G_term_Weight                           = 1.0;
    }
    else if(Mode == SIMULATION){
        walking.trot.SpeedScale(AXIS_X)         = 0.6;
        walking.trot.SpeedScale(AXIS_Y)         = 0.3;
        walking.trot.SpeedScale(AXIS_YAW)       = 0.5;
        
        walking.trot.LimitAcceleration(AXIS_X)  = 0.5;
        walking.trot.LimitAcceleration(AXIS_Y)  = 0.5;
        walking.trot.LimitAcceleration(AXIS_YAW)= 0.5;
        
        walking.trot.ActiveThreshold            = 0.1;
        
        G_term_Weight                           = 0.9990;
    }

    contact.Foot                                << 1, 1, 1, 1;
    contact.Number                              = 4;
            
    walking.Ready                               = false;
    walking.HomePoseDone                        = false;
    walking.UpDown                              = true;
    torque.CTC_Flag                             = false;
    
    osqp.Flag                                   = true; // 여길 죽여도 상관없을듯
 
    printf(C_PURPLE "The Mass of Robot                     =  %f kg\n" C_RESET, BodyKine.m_total);
    
    printf(C_BLUE "The FL Foot Global Position           =  %f\t  %f\t %f \n" C_RESET, FL_Foot.G_RefPos[0], FL_Foot.G_RefPos[1], FL_Foot.G_RefPos[2]);
    printf(C_BLUE "The FR Foot Global Position           =  %f\t %f\t %f \n" C_RESET, FR_Foot.G_RefPos[0], FR_Foot.G_RefPos[1], FR_Foot.G_RefPos[2]);
    printf(C_BLUE "The RL Foot Global Position           = %f\t  %f\t %f \n" C_RESET, RL_Foot.G_RefPos[0], RL_Foot.G_RefPos[1], RL_Foot.G_RefPos[2]);
    printf(C_BLUE "The RR Foot Global Position           = %f\t %f\t %f \n" C_RESET, RR_Foot.G_RefPos[0], RR_Foot.G_RefPos[1], RR_Foot.G_RefPos[2]);
    
    printf(C_GREEN "Robot CoM Init Global Position        = %f\t %f\t %f \n" C_RESET, com.G_InitPos[0], com.G_InitPos[1], com.G_InitPos[2]);
    printf(C_GREEN "Robot CoM Target Global Position      =  %f\t  %f\t %f \n" C_RESET, com.G_RefTargetPos[0], com.G_RefTargetPos[1], com.G_RefTargetPos[2]);        
    free_matrix(OutMat, 1, 3, 1, 3);
}

void CRobot::initializeOSQP(void) {
   /* initializeOSQP
    * Initialize OSQP Setting
    */
    
    static int temp_index                       = 0;      
    static bool temp_flag                       = false;   

    osqp.I_g                                    = BodyKine.G_I_g;
            
    osqp.com.G_RefAcc                             << 0,   0,   0;
    osqp.base.G_RefAngularAcc                     << 0,   0,   0;

    osqp.Mass                                   = BodyKine.m_total;
    osqp.Gravity                                <<  0,  0,  9.81;
    
    if(Mode == ACTUAL_ROBOT){
        gain.CoM_QP_Kp                          <<        80,      0,      0,
                                                           0,     60,      0,
                                                           0,      0,     50;

        gain.CoM_QP_Kd                          <<         2,      0,      0,
                                                           0,      2,      0,
                                                           0,      0,      5;

        gain.BASE_QP_Kp                         <<       400,      0,      0,
                                                           0,    400,      0,
                                                           0,      0,    150;

        gain.BASE_QP_Kd                         <<        30,      0,      0,
                                                           0,     30,      0,
                                                           0,      0,     15;
    }
    else if(Mode == SIMULATION){
        gain.CoM_QP_Kp                          <<        80,      0,      0,
                                                           0,     60,      0,
                                                           0,      0,     50;

        gain.CoM_QP_Kd                          <<         2,      0,      0,
                                                           0,      2,      0,
                                                           0,      0,      5;

        gain.BASE_QP_Kp                         <<       400,      0,      0,
                                                           0,    400,      0,
                                                           0,      0,    150;

        gain.BASE_QP_Kd                         <<        30,      0,      0,
                                                           0,     30,      0,
                                                           0,      0,     15;
    }
    
    gain.InitCoM_QP_Kp                          = gain.CoM_QP_Kp;
    gain.InitCoM_QP_Kd                          = gain.CoM_QP_Kd;
    gain.InitBASE_QP_Kp                         = gain.BASE_QP_Kp;
    gain.InitBASE_QP_Kd                         = gain.BASE_QP_Kd;
    



    // 
    osqp.A.block<3, 3>(0, 0)                    = Matrix3f::Identity(3, 3);
    osqp.A.block<3, 3>(0, 3)                    = Matrix3f::Identity(3, 3);
    osqp.A.block<3, 3>(0, 6)                    = Matrix3f::Identity(3, 3);
    osqp.A.block<3, 3>(0, 9)                    = Matrix3f::Identity(3, 3);
    
    osqp.POS_CoM_CrossProduct                   <<                                                  0,      -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),       FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),           RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),         RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),
                                                             FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,      -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,          -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,        -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),
                                                            -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                              0,      -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                                  0,      -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                                0;

    osqp.A.block<3, 12>(3, 0)                   =   osqp.POS_CoM_CrossProduct;




    //** MPC: A_c
    float phi = imu.Ori(AXIS_ROLL);
    float theta = imu.Ori(AXIS_PITCH);
    float psi = imu.Ori(AXIS_YAW);

    osqp.A_c.block<3, 3>(0, 6) = rot_z(psi).transpose();
    osqp.A_c.block<3, 3>(3, 9) = MatrixXf::Identity(3, 3);
    osqp.A_c(11, 12) = -1.;
    //osqp.A_c(11, 12) = 1.;

    //** MPC: B_c
    // Trot 이 없을 때이므로, 미래 다리의 궤적을 현재 궤적으로 그냥 넣어준다.
    if(walking.phase == TROT_NONE){
        for(size_t i = 0; i < horizontal_length; ++i){
            // osqp.r_fl_vector.block<3, 1>(3*i, 0) = temp_com2FL_ref.block<3, 1>(3*i, 0);  
            // osqp.r_fr_vector.block<3, 1>(3*i, 0) = temp_com2FR_ref.block<3, 1>(3*i, 0);  
            // osqp.r_rl_vector.block<3, 1>(3*i, 0) = temp_com2RL_ref.block<3, 1>(3*i, 0);  
            // osqp.r_rr_vector.block<3, 1>(3*i, 0) = temp_com2RR_ref.block<3, 1>(3*i, 0);

            osqp.r_fl_vector.block<3, 1>(3*i, 0) << 0.298040, 0.198800, -0.4;  
            osqp.r_fr_vector.block<3, 1>(3*i, 0) << 0.298040, -0.198800, -0.4;  
            osqp.r_rl_vector.block<3, 1>(3*i, 0) << -0.298040, 0.198800, -0.4;  
            osqp.r_rr_vector.block<3, 1>(3*i, 0) << -0.298040,- 0.198800, -0.4;  

        }
    }

    MatrixXf G_I_g_diagonal = MatrixXf::Zero(3,3);
    G_I_g_diagonal(0,0) = BodyKine.G_I_g(0,0);
    G_I_g_diagonal(1,1) = BodyKine.G_I_g(1,1);
    G_I_g_diagonal(2,2) = BodyKine.G_I_g(2,2);

    // osqp.w_I_g = rot_z(psi).transpose()*BodyKine.G_I_g*rot_z(psi);
    osqp.w_I_g = rot_z(psi).transpose()*G_I_g_diagonal*rot_z(psi);
    osqp.w_I_g_transpose = osqp.w_I_g.transpose();
    osqp.w_I_g_inverse = osqp.w_I_g.inverse();


    for(size_t i = 0; i < horizontal_length; ++i){
        // B_c 는 13*(3*4) 의 행렬이 l 개의 행으로 구성된 (13*l) * (3*4) 행렬.
        osqp.B_c.block<3, 3>(13*i + 6, 0) = osqp.w_I_g_inverse * skew_matrix(osqp.r_fl_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 3) = osqp.w_I_g_inverse * skew_matrix(osqp.r_fr_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 6) = osqp.w_I_g_inverse * skew_matrix(osqp.r_rl_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 9) = osqp.w_I_g_inverse * skew_matrix(osqp.r_rr_vector.block<3, 1>(3*i, 0));

        // osqp.B_c.block<3, 3>(13*i + 6, 0) = skew_matrix(osqp.r_fl_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 3) = skew_matrix(osqp.r_fr_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 6) = skew_matrix(osqp.r_rl_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 9) = skew_matrix(osqp.r_rr_vector.block<3, 1>(3*i, 0));


        osqp.B_c.block<3, 3>(13*i + 9, 0) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 3) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 6) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 9) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
    }


    //** MPC: Discrete domain A_hat, B_hat
    osqp.A_hat = osqp.A_c * tasktime + MatrixXf::Identity(13, 13);
    osqp.B_hat = osqp.B_c * tasktime;

    //** MPC: A_qp, B_qp
    // A_qp
    for(size_t i = 0; i < horizontal_length; ++i){
        if(i == 0){
            osqp.A_qp.block<13, 13>(13*i, 0) = osqp.A_hat;
        }
        else{
            osqp.A_qp.block<13, 13>(13*i, 0) = osqp.A_hat * osqp.A_qp.block<13, 13>(13*(i-1), 0);
        }
    }

    // B_qp
    for(size_t row = 0; row < horizontal_length; ++row){      // 각 row번째 행에 대해서
        for(size_t col = 0; col < horizontal_length; ++col){  // 각 col번째 열에 대해서
            if(col <= row){
                osqp.B_qp.block<13, 12>(13*row, 12*col) = osqp.B_hat.block<13, 12>(13*col, 0);
            }

            if(row > col){
                osqp.B_qp.block<13, 12>(13*row, 12*col) = osqp.A_qp.block<13, 13>(13*(row-col-1), 0) * osqp.B_qp.block<13, 12>(13*row, 12*col);
            }
        }
    }

    //** MPC: state X_mpc
    osqp.X_mpc.block<3, 1>(0, 0) = base.G_CurrentOri;
    osqp.X_mpc.block<3, 1>(3, 0) = com.G_Foot_to_CoM_CurrentPos;
    osqp.X_mpc.block<3, 1>(6, 0) = base.G_CurrentAngularVel;
    osqp.X_mpc.block<3, 1>(9, 0) = com.G_Foot_to_CoM_CurrentVel;
    osqp.X_mpc(12) = osqp.Gravity(2);

    //** MPC: horizontal references
    for(size_t i = 0; i < horizontal_length; ++i){
        if(walking.phase == TROT_NONE){
            osqp.X_ref_horizon.block<3, 1>(i*13+0, 0) = base.G_RefOri; 
            osqp.X_ref_horizon.block<3, 1>(i*13+3, 0) = com.G_Foot_to_CoM_RefPos;
            osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = base.G_RefAngularVel;
            osqp.X_ref_horizon.block<3, 1>(i*13+9, 0) = com.G_Foot_to_CoM_RefVel;
            osqp.X_ref_horizon(i*13+12) = osqp.Gravity(2);
        }

        else{   // Trot 중일 때는 실제로 미래 궤적들이 계산될테니

            //temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0) = temp_com_ref.block<3, 1>(i*3, 0) - temp_Offset_Global_between_FootCoord;

            osqp.X_ref_horizon.block<3, 1>(i*13+0, 0) = base.G_RefOri;  // 변동 없음
            
            osqp.X_ref_horizon.block<3, 1>(i*13+3, 0) = temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0);
            //osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = base.G_RefAngularVel;
            osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = VectorXf::Zero(3);
            osqp.X_ref_horizon.block<3, 1>(i*13+9, 0) = temp_Foot_to_CoM_vel_ref.block<3, 1>(i*3, 0);
            osqp.X_ref_horizon(i*13+12) = osqp.Gravity(2);
        }

        //temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0)
    }

    //** MPC: L and K weight gain : from MIT paper
    
    // osqp.L_ori = MatrixXf::Identity(3, 3);
    // osqp.L_pos(2, 2) = 50.;
    // osqp.L_d_ori(2, 2) = 1.;
    // osqp.L_d_pos = MatrixXf::Identity(3, 3);


    // // osqp.L_pos(0, 0) = 20.;
    // // osqp.L_pos(1, 1) = 20.;
    // // osqp.L_d_ori(0, 0) = 0.5;
    // // osqp.L_d_ori(1, 1) = 0.5;

    // osqp.weight_alpha = 0.000001;
    // // osqp.weight_alpha = 0.00000001;
    

    //** From previous work
    // osqp.L_ori << 500000000., 0., 0.,
    //             0., 500000000., 0.,
    //             0., 0., 500000000.;

    // osqp.L_pos << 3000000000., 0., 0.,
    //             0., 3000000000., 0.,
    //             0., 0., 7000000000.;

    // osqp.L_d_ori << 50000, 0, 0,
    //                 0., 150000., 0.,
    //                 0., 0., 150000.;

    // osqp.L_d_pos << 70000., 0., 0.,
    //                 0., 70000., 0.,
    //                 0., 0., 70000.;
    
    // osqp.weight_alpha = 0.00001;

    //** 230704
    // osqp.L_ori << 30000000., 0., 0.,
    //             0., 30000000., 0.,
    //             0., 0., 30000000.;

    // osqp.L_ori << 5000000., 0., 0.,
    //             0., 5000000., 0.,
    //             0., 0., 5000000.;

    // osqp.L_pos << 30000000., 0., 0.,
    //             0., 30000000., 0.,
    //             0., 0., 30000000.;

    // osqp.L_d_ori << 150000, 0, 0,
    //                 0., 150000., 0.,
    //                 0., 0., 150000.;

    // osqp.L_d_pos << 70000., 0., 0.,
    //                 0., 70000., 0.,
    //                 0., 0., 70000.;
    
    // // osqp.weight_alpha = 0.00005;
    // osqp.weight_alpha = 0.00001;

    //** 230705
    // osqp.L_ori << 100000., 0., 0.,
    //             0., 1000000., 0.,
    //             0., 0., 100000.;

    // osqp.L_pos << 2800000., 0., 0.,
    //             0., 2800000., 0.,
    //             0., 0., 3200000.;

    // osqp.L_d_ori << 100., 0, 0,
    //                 0., 100., 0.,
    //                 0., 0., 100.;

    // osqp.L_d_pos << 1000., 0., 0.,
    //                 0., 1000., 0.,
    //                 0., 0., 500.;

    // osqp.weight_alpha = 0.001;

    osqp.L_ori << 50., 0., 0.,
                0., 20000., 0.,
                0., 0., 50.;

    osqp.L_pos << 2500., 0., 0.,
                0., 2500., 0.,
                0., 0., 2500.;

    osqp.L_d_ori << 0.001, 0, 0,
                    0., 0.001, 0.,
                    0., 0., 0.001;

    osqp.L_d_pos << 20., 0., 0.,
                    0., 20., 0.,
                    0., 0., 20.;

    osqp.weight_alpha = 0.001;



    for(size_t i = 0; i < horizontal_length; ++i){
        osqp.L_mpc.block<3, 3>(i * 13, i * 13) =            osqp.L_ori;
        osqp.L_mpc.block<3, 3>(i * 13 + 3, i * 13 + 3) =    osqp.L_pos;
        osqp.L_mpc.block<3, 3>(i * 13 + 6, i * 13 + 6) =    osqp.L_d_ori;
        osqp.L_mpc.block<3, 3>(i * 13 + 9, i * 13 + 9) =    osqp.L_d_pos;
        osqp.L_mpc(i * 13 + 12, i * 13 + 12) = 1.;  // maybe?        
    }
    osqp.K_mpc *= osqp.weight_alpha;


    // Diagonal matrix vector
    osqp.L_mpc_vec = osqp.L_mpc.diagonal();
    osqp.K_mpc_vec = osqp.K_mpc.diagonal();


    //** osqp.P_mpc & osqp.Q_mpc;
    osqp.P_mpc = 2 * (osqp.B_qp.transpose() * osqp.L_mpc * osqp.B_qp + osqp.K_mpc);
    osqp.q_mpc = 2 * osqp.B_qp.transpose() * osqp.L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);


    // ===================== OSQP  ====================== //
    int row_index = 0;
    int row_index_max = 0;
    int col_index = 0;

    // ===================== P_x ====================== //
    //    c_int P_nnz = 0;
    int OSQP_num1 = 0;
    int OSQP_num2 = 0;
    int OSQP_num3 = 0;

   
    // 5. osqp QP form 의 P에 대한 행렬들을 위에서 만든 osqp.P 행렬을 이용하여 만들어줄 것(희소행렬로 표현해주기 위한 성분들을 만들 것)
    // ===================== P_x ====================== //    
    //    osqp.data_P_x[]
    for (int j = 0; j < osqp.P_mpc.cols(); ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i <= j; ++i) {
            if (osqp.P_mpc(i, j) != 0) {
                osqp.P_x_mpc[OSQP_num1] = osqp.P_mpc(i, j);
                osqp.P_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.P_p_mpc[j + 1] = OSQP_num3;
    }
    //    std::cout << "B" << std::endl;
    // ===================== P_i ====================== //
    //osqp.data_P_i[]


    // ===================== P_p ====================== //
    //osqp.data_P_p[]

    // ===================== q ====================== //
    //osqp.data_q[i]
    for (unsigned int i = 0; i < osqp.P.cols(); ++i) {
        osqp.data_q[i] = osqp.q(i);
    }
    //    c_float q_1[horizontal_length * 12];
    // ===================== q ====================== //
    for (unsigned int i = 0; i < horizontal_length * 12; ++i) {
        osqp.q_mpc_[i] = osqp.q_mpc(i);
    }
    //    std::cout << "C" << std::endl;


    //* WsJi - A Matrix prepare
    Vector3f n_(0, 0, 1);
    Vector3f t1_(1, 0, 0);
    Vector3f t2_(0, 1, 0);

    float fric_coef = 0.8;

    MatrixXf C1_(5, 3), C2_(5, 3), C3_(5, 3), C4_(5, 3);
    C1_.block(0, 0, 1, 3) = (-fric_coef * n_ + t1_).transpose();
    C1_.block(1, 0, 1, 3) = (-fric_coef * n_ + t2_).transpose();
    C1_.block(2, 0, 1, 3) = (fric_coef * n_ + t2_).transpose();
    C1_.block(3, 0, 1, 3) = (fric_coef * n_ + t1_).transpose();
    C1_.block(4, 0, 1, 3) = (n_).transpose();

    C2_ = C1_;
    C3_ = C1_;
    C4_ = C1_;

    osqp.C_.block(0, 0, 5, 3) = C1_;
    osqp.C_.block(5, 3, 5, 3) = C2_;
    osqp.C_.block(10, 6, 5, 3) = C3_;
    osqp.C_.block(15, 9, 5, 3) = C4_;


    //** MPC horizontal for osqp.C
    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.C_mpc.block(i * 20, i * 12, 20, 12) = osqp.C_;
    }

   
    OSQP_num1 = 0;
    OSQP_num2 = 0;
    OSQP_num3 = 0;
    for (int j = 0; j < 12 * horizontal_length; ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i < 20 * horizontal_length; ++i) {
            if (osqp.C_mpc(i, j) != 0) {
                osqp.A_x_mpc[OSQP_num1] = osqp.C_mpc(i, j);
                osqp.A_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.A_p_mpc[j + 1] = OSQP_num3;
        //        std::cout << "OSQP_A_p[" << j+1<< "]=" <<osqp.A_p[j + 1] << std::endl;
    }
    //        std::cout << "D" << std::endl;
    //////////////

    // RL constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i] = -10000.;
        osqp.data_l_mpc[20 * i + 1] = -10000.;
        osqp.data_l_mpc[20 * i + 2] = 0.;
        osqp.data_l_mpc[20 * i + 3] = 0.;
        osqp.data_l_mpc[20 * i + 4] = 10.;
        osqp.data_u_mpc[20 * i] = 0.;
        osqp.data_u_mpc[20 * i + 1] = 0.;
        osqp.data_u_mpc[20 * i + 2] = 10000.;
        osqp.data_u_mpc[20 * i + 3] = 10000.;
        osqp.data_u_mpc[20 * i + 4] = 400.;


        if(walking.phase != TROT_NONE){
            if(temp_contact_foots(0, i) == false){
                osqp.data_l_mpc[20 * i + 4] = -10000.;
                osqp.data_u_mpc[20 * i + 4] = 10000.;
            }
        }
    }
    // RR constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 5] = -10000.;
        osqp.data_l_mpc[20 * i + 6] = -10000.;
        osqp.data_l_mpc[20 * i + 7] = 0.;
        osqp.data_l_mpc[20 * i + 8] = 0.;
        osqp.data_l_mpc[20 * i + 9] = 10.;
        osqp.data_u_mpc[20 * i + 5] = 0.;
        osqp.data_u_mpc[20 * i + 6] = 0.;
        osqp.data_u_mpc[20 * i + 7] = 10000.;
        osqp.data_u_mpc[20 * i + 8] = 10000.;
        osqp.data_u_mpc[20 * i + 9] = 400.;

        if(walking.phase != TROT_NONE){
            if(temp_contact_foots(1, i) == false){
                osqp.data_l_mpc[20 * i + 9] = -10000.;
                osqp.data_u_mpc[20 * i + 9] = 10000.;
            }
        }
    }
    // FL constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 10] = -10000.;
        osqp.data_l_mpc[20 * i + 11] = -10000.;
        osqp.data_l_mpc[20 * i + 12] = 0.;
        osqp.data_l_mpc[20 * i + 13] = 0.;
        osqp.data_l_mpc[20 * i + 14] = 10.;
        osqp.data_u_mpc[20 * i + 10] = 0.;
        osqp.data_u_mpc[20 * i + 11] = 0.;
        osqp.data_u_mpc[20 * i + 12] = 10000.;
        osqp.data_u_mpc[20 * i + 13] = 10000.;
        osqp.data_u_mpc[20 * i + 14] = 400.;

        if(walking.phase != TROT_NONE){
            if(temp_contact_foots(2, i) == false){
                osqp.data_l_mpc[20 * i + 14] = -10000.;
                osqp.data_u_mpc[20 * i + 14] = 10000.;
            }
        }
    }
    // FR constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 15] = -10000.;
        osqp.data_l_mpc[20 * i + 16] = -10000.;
        osqp.data_l_mpc[20 * i + 17] = 0.;
        osqp.data_l_mpc[20 * i + 18] = 0.;
        osqp.data_l_mpc[20 * i + 19] = 10.;
        osqp.data_u_mpc[20 * i + 15] = 0.;
        osqp.data_u_mpc[20 * i + 16] = 0.;
        osqp.data_u_mpc[20 * i + 17] = 10000.;
        osqp.data_u_mpc[20 * i + 18] = 10000.;
        osqp.data_u_mpc[20 * i + 19] = 400.;

        if(walking.phase != TROT_NONE){
            if(temp_contact_foots(3, i) == false){
                osqp.data_l_mpc[20 * i + 19] = -10000.;
                osqp.data_u_mpc[20 * i + 19] = 10000.;
            }
        }
    }


    if (OSQP_data) {
        OSQP_data->n = 12 * horizontal_length;
        OSQP_data->m = 20 * horizontal_length;
        OSQP_data->P = csc_matrix(OSQP_data->n, OSQP_data->n, osqp.P_nnz_mpc, osqp.P_x_mpc, osqp.P_i_mpc, osqp.P_p_mpc);
        OSQP_data->q = osqp.q_mpc_;
        OSQP_data->A = csc_matrix(OSQP_data->m, OSQP_data->n, osqp.A_nnz_mpc, osqp.A_x_mpc, osqp.A_i_mpc, osqp.A_p_mpc);
        OSQP_data->l = osqp.data_l_mpc;
        OSQP_data->u = osqp.data_u_mpc;
    }
    //    delete [] OSQP_A_x;
    //    free(OSQP_A_x);
    // Define solver settings as default
    if (OSQP_settings) {
        osqp_set_default_settings(OSQP_settings);
        OSQP_settings->alpha = 1; // Change alpha parameter
        OSQP_settings->verbose = false;
        // OSQP_settings->verbose = true;

        //eps_abs 와 eps_rel 의 기본값은 1e-03 인데 이를 1e-06 으로 조금 더 엄격하게 수정해본다.
        OSQP_settings->eps_abs = 1e-03;
        OSQP_settings->eps_rel = 1e-03;
    }
    osqp_cleanup(OSQP_work);
    //    std::cout << "E" << std::endl;
    // Setup workspace
    osqp.exitflag = osqp_setup(&OSQP_work, OSQP_data, OSQP_settings);
    //    std::cout << "F" << std::endl;
    // Solve Problem
    osqp_solve(OSQP_work);



    std::cout << "A_c: \n" << osqp.A_c << std::endl;
    std::cout << "A_qp: \n" << osqp.A_qp << std::endl;

    std::cout << "B_c: \n" << osqp.B_c << std::endl;
    std::cout << "B_qp: \n" << osqp.B_qp << std::endl;

    std::cout << "osqp.X_mpc: \n" << osqp.X_mpc << std::endl;
    std::cout << "osqp.X_ref_horizon: \n" << osqp.X_ref_horizon << std::endl;








 #if 0   
    //
    osqp.b                                      <<  osqp.Mass   *   (osqp.com.G_RefAcc + osqp.Gravity),
                                                    osqp.I_g    *   osqp.base.G_RefAngularAcc;
    
    osqp.S(0,0)                                 = 5.;                                        
    osqp.S(1,1)                                 = 5.;
    osqp.S(2,2)                                 = 10.;
                                    
    osqp.S(3,3)                                 = 10.;                                        
    osqp.S(4,4)                                 = 10.;
    osqp.S(5,5)                                 = 10.;

    for(size_t i = 0; i < 4; ++i){
        // GRF_X
        osqp.W(i*3, i*3)                        = 1*20.0;  // 1*20
        // GRF_Y                   
        osqp.W(i*3+1, i*3+1)                    = 1*20.0;  // 1*20
        // GRF_Z                   
        osqp.W(i*3+2, i*3+2)                    = 1*1.0;  // 1*1.0
    }

    osqp.P                                      =  2.0*((osqp.A.transpose() * osqp.S* osqp.A) + (osqp.W * osqp.alpha));
    osqp.q                                      = -2.0* osqp.b.transpose() * osqp.S* osqp.A;
        
    osqp.data_P_nnz                             = 0;
    osqp.data_P_p[13]                           = {0, };
                
    temp_index                                  = 0;
    temp_flag                                   = false;
    
    // ===================== P_x P_i ====================== //
    for (int j = 0; j < osqp.P.cols(); ++j) {
        for (int i = 0; i <= j; ++i) {
            if (osqp.P(i, j) != 0) {
                osqp.data_P_x[temp_index] = osqp.P(i, j);
                osqp.data_P_i[temp_index] = i;
                osqp.data_P_nnz++;
                
                if(!temp_flag){
                    temp_flag = true;
                    osqp.data_P_p[j] = temp_index;
                }
                ++temp_index;
            }
        }
        temp_flag = false;
    }
    osqp.data_P_p[osqp.P.cols()] = temp_index;
   
    // ===================== q ====================== //
    temp_index                          = 0;
    
    for (unsigned int i = 0; i < osqp.q.rows(); ++i) {
            osqp.data_q[temp_index] = osqp.q(i);
            ++temp_index;
    }
    
    osqp.tangential_direction1_local            << 1, 0, 0;
    osqp.tangential_direction2_local            << 0, 1, 0;
    osqp.SurfaceNormal_local                    << 0, 0, 1;
    
    osqp.tangential_direction1                  = osqp.tangential_direction1_local;
    osqp.tangential_direction2                  = osqp.tangential_direction2_local;
    osqp.SurfaceNormal                          = osqp.SurfaceNormal_local;
    
    
    MatrixXf C1_(5, 3), C2_(5,3), C3_(5,3), C4_(5,3);
    C1_.block(0,0,1,3)                          = (-osqp.mu*osqp.SurfaceNormal + osqp.tangential_direction1).transpose();
    C1_.block(1,0,1,3)                          = (-osqp.mu*osqp.SurfaceNormal + osqp.tangential_direction2).transpose();
    C1_.block(2,0,1,3)                          = ( osqp.mu*osqp.SurfaceNormal + osqp.tangential_direction2).transpose();
    C1_.block(3,0,1,3)                          = ( osqp.mu*osqp.SurfaceNormal + osqp.tangential_direction1).transpose();
    C1_.block(4,0,1,3)                          = (osqp.SurfaceNormal).transpose();

    C2_                                         = C1_;
    C3_                                         = C1_;
    C4_                                         = C1_;
        
    osqp.C_.block(0,0,5,3)                      = C1_;
    osqp.C_.block(5,3,5,3)                      = C2_;
    osqp.C_.block(10,6,5,3)                     = C3_;
    osqp.C_.block(15,9,5,3)                     = C4_;
            
    temp_index                                  = 0;
    temp_flag                                   = false;  
    
    osqp.data_A_nnz                             = 0;
    osqp.data_A_p[13]                           = {0, };
    // ===================== A_nnz ====================== //
    
    for (int j = 0; j < osqp.C_.cols(); ++j) {
        for (int i = 0; i < osqp.C_.rows(); ++i) {
            if (osqp.C_(i, j) != 0) {
                osqp.data_A_x[temp_index]       = osqp.C_(i, j);
                osqp.data_A_i[temp_index]       = i;
                osqp.data_A_nnz++;
                
                if(!temp_flag){
                    temp_flag = true;
                    osqp.data_A_p[j]            = temp_index;
                }
                ++temp_index;
            }
        }
        temp_flag = false;
    }
    osqp.data_A_p[osqp.C_.cols()]               = temp_index;
    
    for(int i = 0; i < 4; ++i){
        osqp.data_l[5*i + 0]                    = -10000.;
        osqp.data_l[5*i + 1]                    = -10000.;
        osqp.data_l[5*i + 2]                    =      0.;
        osqp.data_l[5*i + 3]                    =      0.;
        osqp.data_l[5*i + 4]                    =      0.;
                      
        osqp.data_u[5*i + 0]                    =      0.;
        osqp.data_u[5*i + 1]                    =      0.;
        osqp.data_u[5*i + 2]                    =  10000.;
        osqp.data_u[5*i + 3]                    =  10000.;
        osqp.data_u[5*i + 4]                    =    500.;
    }
    
    osqp.data_m                                 = 20;
    osqp.data_n                                 = 12;
    
    //Populate data
    if (OSQP_data){
        OSQP_data->n                            = osqp.data_n;
        OSQP_data->m                            = osqp.data_m;
        OSQP_data->P                            = csc_matrix(OSQP_data->n, OSQP_data->n, osqp.data_P_nnz, osqp.data_P_x, osqp.data_P_i, osqp.data_P_p);
        OSQP_data->q                            = osqp.data_q;
        OSQP_data->A                            = csc_matrix(OSQP_data->m, OSQP_data->n, osqp.data_A_nnz, osqp.data_A_x, osqp.data_A_i, osqp.data_A_p);
        OSQP_data->l                            = osqp.data_l;
        OSQP_data->u                            = osqp.data_u;
    }
    
    // Define solver settings as default
    if (OSQP_settings){
        osqp_set_default_settings(OSQP_settings);
        OSQP_settings->alpha                    = 1; // Change alpha parameter

        // 이 문장을 추가해주면 프린트가 멈출 걸?
        OSQP_settings->verbose = false;
    }

    // Setup workspace
    osqp.exitflag                               = osqp_setup(&OSQP_work, OSQP_data, OSQP_settings);
    
    // Solve Problem
    osqp_solve(OSQP_work);
    
    #endif
}

void CRobot::ProcessOSQP(void) {   
   /* ProcessOSQP
    * Calculate Optimal Reference GRF(Ground Reaction Force)
    */
    
    static int temp_index                       = 0;      
    static bool temp_flag                       = false;    
    
    #if 0
    osqp.I_g                                    = BodyKine.G_I_g;

    osqp.POS_CoM_CrossProduct                       <<                                                  0,      -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),       FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),           RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),                                              0,      -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),         RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),
                                                                 FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,      -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,          -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),       RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Z),                                              0,        -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),
                                                                -FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                              0,      -FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                              0,      -RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                                  0,      -RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_Y),       RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS_X),                                                0;
    
    osqp.A.block<3, 12>(3, 0)                   =   osqp.POS_CoM_CrossProduct;   
    
    osqp.com.G_RefAcc                             = gain.CoM_QP_Kp  * (com.G_Foot_to_CoM_RefPos - com.G_Foot_to_CoM_CurrentPos)\
                                                  + gain.CoM_QP_Kd  * (com.G_Foot_to_CoM_RefVel - com.G_Foot_to_CoM_CurrentVel);
    
    osqp.base.G_RefAngularAcc                     = gain.BASE_QP_Kp * RotMat2EulerZYX((EulerZYX2RotMat(base.G_RefOri)*(EulerZYX2RotMat(base.G_CurrentOri)).transpose()))\
                                                  + gain.BASE_QP_Kd * (base.G_RefAngularVel - base.G_CurrentAngularVel);
            
    osqp.b                                      <<  osqp.Mass   *   (osqp.com.G_RefAcc + osqp.Gravity), 
                                                    osqp.I_g    *   (osqp.base.G_RefAngularAcc);
        
    osqp.P                                      =  2.*((osqp.A.transpose() * osqp.S * osqp.A) + (osqp.alpha * osqp.W));
    osqp.q                                      = -2.*  osqp.b.transpose() * osqp.S * osqp.A;
        
    osqp.data_P_nnz                             = 0;    
    osqp.data_P_p[13]                           = {0, };
            
    temp_index                                  = 0;        
    temp_flag                                   = false;     
    
    // ===================== P_x P_i ====================== //
    for (int j = 0; j < osqp.P.cols(); ++j) {
        for (int i = 0; i <= j; ++i) {
            if (osqp.P(i, j) != 0) {
                osqp.data_P_x[temp_index]       = osqp.P(i, j);
                osqp.data_P_i[temp_index]       = i;
                osqp.data_P_nnz++;
            
                if(!temp_flag){
                    temp_flag = true;
                    osqp.data_P_p[j]            = temp_index;
                }
                ++temp_index;
            }
        }
        temp_flag = false;
    }
    
    osqp.data_P_p[osqp.P.cols()]                = temp_index;
    
    // ===================== q ====================== //
    temp_index                          = 0;
        
    for (unsigned int i = 0; i < osqp.q.rows(); ++i) {
        osqp.data_q[temp_index] = osqp.q(i);
        ++temp_index;
    }       

    for (unsigned int i = 0; i < 4; ++i) {
        if(contact.Foot(i) == 0){
            osqp.data_l[5*i + 0]                =      0.;
            osqp.data_l[5*i + 1]                =      0.;
            osqp.data_l[5*i + 2]                =      0.;
            osqp.data_l[5*i + 3]                =      0.;
            osqp.data_l[5*i + 4]                =      0.;
                      
            osqp.data_u[5*i + 0]                =      0.;
            osqp.data_u[5*i + 1]                =      0.;
            osqp.data_u[5*i + 2]                =      0.;
            osqp.data_u[5*i + 3]                =      0.;
            osqp.data_u[5*i + 4]                =      0.; 
        }          
        else{          
            osqp.data_l[5*i + 0]                = -10000.;
            osqp.data_l[5*i + 1]                = -10000.;
            osqp.data_l[5*i + 2]                =      0.;
            osqp.data_l[5*i + 3]                =      0.;
            osqp.data_l[5*i + 4]                =      0.; 
                      
            osqp.data_u[5*i + 0]                =      0.;
            osqp.data_u[5*i + 1]                =      0.;
            osqp.data_u[5*i + 2]                =  10000.;
            osqp.data_u[5*i + 3]                =  10000.;
            osqp.data_u[5*i + 4]                =    500.;
        }
    }
    
    // Update problem
    osqp_update_P(OSQP_work, osqp.data_P_x, OSQP_NULL, osqp.data_P_nnz);
    osqp_update_lin_cost(OSQP_work, osqp.data_q);
    osqp_update_bounds(OSQP_work, osqp.data_l, osqp.data_u);
    
    // Solve Problem
    osqp_solve(OSQP_work);

    #endif

    std::chrono::system_clock::time_point calStartTime = std::chrono::system_clock::now();
    


    //**
    //** MPC: A_c
    float phi = imu.Ori(AXIS_ROLL);
    float theta = imu.Ori(AXIS_PITCH);
    float psi = imu.Ori(AXIS_YAW);

    osqp.A_c.block<3, 3>(0, 6) = rot_z(psi).transpose();
    osqp.A_c.block<3, 3>(3, 9) = MatrixXf::Identity(3, 3);
    osqp.A_c(11, 12) = -1;

    //** MPC: B_c
    // Trot 이 없을 때이므로, 미래 다리의 궤적을 현재 궤적으로 그냥 넣어준다.

    // osqp.r_fl_vector.block<3, 1>(3*i, 0) << 0.298040, 0.198800, -0.4;  
    // osqp.r_fr_vector.block<3, 1>(3*i, 0) << 0.298040, -0.198800, -0.4;  
    // osqp.r_rl_vector.block<3, 1>(3*i, 0) << -0.298040, 0.198800, -0.4;  
    // osqp.r_rr_vector.block<3, 1>(3*i, 0) << -0.298040,- 0.198800, -0.4; 

    if(walking.phase == TROT_NONE){
        for(size_t i = 0; i < horizontal_length; ++i){

           

            // osqp.r_fl_vector.block<3, 1>(3*i, 0) = temp_com2FL_ref.block<3, 1>(3*i, 0);  
            // osqp.r_fr_vector.block<3, 1>(3*i, 0) = temp_com2FR_ref.block<3, 1>(3*i, 0);  
            // osqp.r_rl_vector.block<3, 1>(3*i, 0) = temp_com2RL_ref.block<3, 1>(3*i, 0);  
            // osqp.r_rr_vector.block<3, 1>(3*i, 0) = temp_com2RR_ref.block<3, 1>(3*i, 0); 
            // if(i == 0){
            //     osqp.r_fl_vector.block<3, 1>(3*i, 0) = FL_Foot.G_CoM_to_Foot_CurrentPos;   // 글로벌 좌표계에서 바라본 몸체로부터 (왼쪽 앞)발 끝까지의 위치 벡터
            //     osqp.r_fr_vector.block<3, 1>(3*i, 0) = FR_Foot.G_CoM_to_Foot_CurrentPos;
            //     osqp.r_rl_vector.block<3, 1>(3*i, 0) = RL_Foot.G_CoM_to_Foot_CurrentPos;
            //     osqp.r_rr_vector.block<3, 1>(3*i, 0) = RR_Foot.G_CoM_to_Foot_CurrentPos;                            
            // }
            // osqp.r_fl_vector.block<3, 1>(3*i, 0) = FL_Foot.G_CoM_to_Foot_CurrentPos;  
            // osqp.r_fr_vector.block<3, 1>(3*i, 0) = FR_Foot.G_CoM_to_Foot_CurrentPos;  
            // osqp.r_rl_vector.block<3, 1>(3*i, 0) = RL_Foot.G_CoM_to_Foot_CurrentPos;  
            // osqp.r_rr_vector.block<3, 1>(3*i, 0) = RR_Foot.G_CoM_to_Foot_CurrentPos;

            osqp.r_fl_vector.block<3, 1>(3*i, 0) = FL_Foot.B_CoM_to_Foot_CurrentPos;  
            osqp.r_fr_vector.block<3, 1>(3*i, 0) = FR_Foot.B_CoM_to_Foot_CurrentPos;  
            osqp.r_rl_vector.block<3, 1>(3*i, 0) = RL_Foot.B_CoM_to_Foot_CurrentPos;  
            osqp.r_rr_vector.block<3, 1>(3*i, 0) = RR_Foot.B_CoM_to_Foot_CurrentPos;

            if(i>0){
                // osqp.r_fl_vector.block<3, 1>(3*i, 0) << 0.298040, 0.198800, -0.4;    // 0.340, 0.212, -0.4
                // osqp.r_fr_vector.block<3, 1>(3*i, 0) << 0.298040, -0.198800, -0.4;  
                // osqp.r_rl_vector.block<3, 1>(3*i, 0) << -0.298040, 0.198800, -0.4;  
                // osqp.r_rr_vector.block<3, 1>(3*i, 0) << -0.298040,- 0.198800, -0.4; 
                osqp.r_fl_vector.block<3, 1>(3*i, 0) << 0.340, 0.212, -0.4;
                osqp.r_fr_vector.block<3, 1>(3*i, 0) << 0.340, -0.212, -0.4;
                osqp.r_rl_vector.block<3, 1>(3*i, 0) << -0.340, 0.212, -0.4;
                osqp.r_rr_vector.block<3, 1>(3*i, 0) << -0.340, -0.212, -0.4;
            }  
        }
    }

    //** walking phase 가 바뀔 경우
    else{
        for(size_t i = 0; i < horizontal_length; ++i){
            
            // osqp.r_fl_vector.block<3, 1>(3*i, 0) = temp_com2FL_ref.block<3, 1>(3*i*int(mpc_sampletime/tasktime), 0);
            // osqp.r_fr_vector.block<3, 1>(3*i, 0) = temp_com2FR_ref.block<3, 1>(3*i*int(mpc_sampletime/tasktime), 0);
            // osqp.r_rl_vector.block<3, 1>(3*i, 0) = temp_com2RL_ref.block<3, 1>(3*i*int(mpc_sampletime/tasktime), 0);
            // osqp.r_rr_vector.block<3, 1>(3*i, 0) = temp_com2RR_ref.block<3, 1>(3*i*int(mpc_sampletime/tasktime), 0);
        }
    }
    
    // osqp.w_I_g = rot_z(psi).transpose()*BodyKine.G_I_g*rot_z(psi);
    // osqp.w_I_g_transpose = osqp.w_I_g.transpose();
    // osqp.w_I_g_inverse = osqp.w_I_g.inverse();

    MatrixXf G_I_g_diagonal = MatrixXf::Zero(3,3);  // 윤호: Diagonal 이 아니라, 실제 텐서를 쓰는 게 맞을 수 있다.
    G_I_g_diagonal(0,0) = BodyKine.G_I_g(0,0);
    G_I_g_diagonal(1,1) = BodyKine.G_I_g(1,1);
    G_I_g_diagonal(2,2) = BodyKine.G_I_g(2,2);

    // osqp.w_I_g = rot_z(psi).transpose()*BodyKine.G_I_g*rot_z(psi);
    
    osqp.w_I_g = rot_z(psi).transpose()*G_I_g_diagonal*rot_z(psi);  // 윤호: 논문의 Global Inertia tensor 식이 틀렸을 수 있다.
    
    // osqp.w_I_g = rot_z(psi)*G_I_g_diagonal*(rot_z(psi).transpose());
    
    osqp.w_I_g_transpose = osqp.w_I_g.transpose();
    osqp.w_I_g_inverse = osqp.w_I_g.inverse();


    for(size_t i = 0; i < horizontal_length; ++i){
        // B_c 는 13*(3*4) 의 행렬이 l 개의 행으로 구성된 (13*l) * (3*4) 행렬.
        osqp.B_c.block<3, 3>(13*i + 6, 0) = osqp.w_I_g_inverse * skew_matrix(osqp.r_fl_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 3) = osqp.w_I_g_inverse * skew_matrix(osqp.r_fr_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 6) = osqp.w_I_g_inverse * skew_matrix(osqp.r_rl_vector.block<3, 1>(3*i, 0));
        osqp.B_c.block<3, 3>(13*i + 6, 9) = osqp.w_I_g_inverse * skew_matrix(osqp.r_rr_vector.block<3, 1>(3*i, 0));

        // osqp.B_c.block<3, 3>(13*i + 6, 0) = skew_matrix(osqp.r_fl_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 3) = skew_matrix(osqp.r_fr_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 6) = skew_matrix(osqp.r_rl_vector.block<3, 1>(3*i, 0));
        // osqp.B_c.block<3, 3>(13*i + 6, 9) = skew_matrix(osqp.r_rr_vector.block<3, 1>(3*i, 0));


        osqp.B_c.block<3, 3>(13*i + 9, 0) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 3) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 6) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
        osqp.B_c.block<3, 3>(13*i + 9, 9) = (1./osqp.Mass)*MatrixXf::Identity(3, 3);
    }


    //** MPC: Discrete domain A_hat, B_hat
    // osqp.A_hat = osqp.A_c * tasktime + MatrixXf::Identity(13, 13);
    // osqp.B_hat = osqp.B_c * tasktime;

    osqp.A_hat = osqp.A_c * mpc_sampletime + MatrixXf::Identity(13, 13); //mpc_sampletime
    osqp.B_hat = osqp.B_c * mpc_sampletime;

    //** 연산 시간이 많이 걸릴 것으로 가장 크게 의심중인 부분
    // std::chrono::system_clock::time_point calTimeAqpBqpStart = std::chrono::system_clock::now();
    // auto calDiffTimebeforeAqpBqp = std::chrono::duration_cast<std::chrono::microseconds>(calTimeAqpBqpStart - calStartTime);
    // std::cout << "Time before calculate A_qp and B_qp: " << calDiffTimebeforeAqpBqp.count() << " [us]" << std::endl;
    

    //** MPC: A_qp, B_qp
    // A_qp
    for(size_t i = 0; i < horizontal_length; ++i){
        if(i == 0){
            osqp.A_qp.block<13, 13>(13*i, 0) = osqp.A_hat;
        }
        else{
            osqp.A_qp.block<13, 13>(13*i, 0) = osqp.A_hat * osqp.A_qp.block<13, 13>(13*(i-1), 0);
        }
    }

    // B_qp
    for(size_t row = 0; row < horizontal_length; ++row){      // 각 row번째 행에 대해서
        for(size_t col = 0; col < horizontal_length; ++col){  // 각 col번째 열에 대해서
            if(col <= row){
                osqp.B_qp.block<13, 12>(13*row, 12*col) = osqp.B_hat.block<13, 12>(13*col, 0);
            }

            if(row > col){
                osqp.B_qp.block<13, 12>(13*row, 12*col) = osqp.A_qp.block<13, 13>(13*(row-col-1), 0) * osqp.B_qp.block<13, 12>(13*row, 12*col);
            }
        }
    }

    ////
    // std::chrono::system_clock::time_point calTimeAqpBqpEnd = std::chrono::system_clock::now();
    // auto calDiffTimeAqpBqp = std::chrono::duration_cast<std::chrono::microseconds>(calTimeAqpBqpEnd - calTimeAqpBqpStart);
    //calEndTime = std::chrono::system_clock::now();
    //std::cout << "calStartTime: " << calStartTime << "\ncalEndTime: " << calEndTime << "\ndifference: " << calEndTime - calStartTime << std::endl;
    // std::cout << "Time for calculate A_qp and B_qp: " << calDiffTimeAqpBqp.count() << " [us]" << std::endl;
    ////



    //** MPC: state X_mpc :: 한번 z 축 이외의 것들을 없애버리면 어떨까
    osqp.X_mpc.block<3, 1>(0, 0) = base.G_CurrentOri;
    

    osqp.X_mpc.block<3, 1>(6, 0) = base.G_CurrentAngularVel; //imu.AngularVel
    osqp.X_mpc(12) = osqp.Gravity(2);


    //** 가제보에서 제공하는 데이터
    // osqp.X_mpc.block<3, 1>(3, 0) = base.G_GroundTruthPos;
    // osqp.X_mpc.block<3, 1>(9, 0) = base.G_GroundTruthVel;


    //** 다리 좌표계로부터 계산되는 데이터
    osqp.X_mpc.block<3, 1>(3, 0) = com.G_Foot_to_CoM_CurrentPos;   
    osqp.X_mpc.block<3, 1>(9, 0) = com.G_Foot_to_CoM_CurrentVel;
    // osqp.X_mpc.block<3, 1>(9, 0) = base.G_Foot_to_Base_CurrentVel;


    // osqp.X_mpc.block<3, 1>(9, 0) = VectorXf::Zero(3);
    

    // osqp.X_mpc.block<3, 1>(0, 0) = VectorXf::Zero(3);
    // osqp.X_mpc.block<3, 1>(3, 0) = com.G_Foot_to_CoM_CurrentPos;
    // //osqp.X_mpc.block<3, 1>(6, 0) = base.G_CurrentAngularVel; //imu.AngularVel
    // osqp.X_mpc.block<3, 1>(6, 0) = VectorXf::Zero(3); //imu.AngularVel
    // osqp.X_mpc.block<3, 1>(9, 0) = VectorXf::Zero(3);
    // osqp.X_mpc(12) = osqp.Gravity(2);


    // base.G_CurrentAngularVel 과 com.G_Foot_to_CoM_CurrentVel 이 nan 으로 뜨는 듯 한데...why?
    for(size_t i = 6; i < 12; ++i){
        if(isnanf(osqp.X_mpc(i)) != 0 ){
            osqp.X_mpc(i) = 0.;
        }
    }

    //** MPC: horizontal references
    for(size_t i = 0; i < horizontal_length; ++i){
        if(walking.phase == TROT_NONE){
            // osqp.X_ref_horizon.block<3, 1>(i*13+0, 0) = base.G_RefOri; 
            osqp.X_ref_horizon.block<3, 1>(i*13+0, 0) = VectorXf::Zero(3); 
            osqp.X_ref_horizon.block<3, 1>(i*13+3, 0) = com.G_Foot_to_CoM_RefPos;
            //osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = base.G_RefAngularVel;
            osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = VectorXf::Zero(3);
            // osqp.X_ref_horizon.block<3, 1>(i*13+9, 0) = com.G_Foot_to_CoM_RefVel;
            osqp.X_ref_horizon.block<3, 1>(i*13+9, 0) = VectorXf::Zero(3);
            osqp.X_ref_horizon(i*13+12) = osqp.Gravity(2);
        }

        else{   // Trot 중일 때는 실제로 미래 궤적들이 계산될테니

            //temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0) = temp_com_ref.block<3, 1>(i*3, 0) - temp_Offset_Global_between_FootCoord;

            osqp.X_ref_horizon.block<3, 1>(i*13+0, 0) = base.G_RefOri;  // 변동 없음
            
            osqp.X_ref_horizon.block<3, 1>(i*13+3, 0) = temp_Foot_to_CoM_ref.block<3, 1>(i*3*int(mpc_sampletime/tasktime), 0);
            //osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = base.G_RefAngularVel;
            osqp.X_ref_horizon.block<3, 1>(i*13+6, 0) = VectorXf::Zero(3);
            osqp.X_ref_horizon.block<3, 1>(i*13+9, 0) = temp_Foot_to_CoM_vel_ref.block<3, 1>(i*3*int(mpc_sampletime/tasktime), 0);
            osqp.X_ref_horizon(i*13+12) = osqp.Gravity(2);
        }

        //temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0)
    }

    // std::cout << "osqp.X_mpc: \n" << osqp.X_mpc << std::endl;
    // std::cout << "osqp.X_ref_horizon(0):\n" << osqp.X_ref_horizon.block<13, 1>(0, 0) << std::endl;

    //**
    ////
    // std::chrono::system_clock::time_point calTimeXref = std::chrono::system_clock::now();
    // auto calDiffTimeXref = std::chrono::duration_cast<std::chrono::microseconds>(calTimeXref - calTimeAqpBqpEnd);
    //calEndTime = std::chrono::system_clock::now();
    //std::cout << "calStartTime: " << calStartTime << "\ncalEndTime: " << calEndTime << "\ndifference: " << calEndTime - calStartTime << std::endl;
    // std::cout << "Time for calculate X_mpc and X_ref: " << calDiffTimeXref.count() << " [us]" << std::endl;
    ////

    


    //** osqp.P_mpc & osqp.Q_mpc; // 여기가 시간이 제일 많이 걸리는 알고리즘이라는 것을 확인하였음.. transpose 때문일까? transpose 를 미리 만들어보면 어떤 결과가 나올지 보자
    osqp.B_qp_transpose = osqp.B_qp.transpose();

    // osqp.P_mpc = 2 * (osqp.B_qp.transpose() * osqp.L_mpc * osqp.B_qp + osqp.K_mpc);
    // osqp.q_mpc = 2 * osqp.B_qp.transpose() * osqp.L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);

    //** 알고리즘 1. 기존 Eigen3 이용해서
    // osqp.P_mpc = 2. * (osqp.B_qp_transpose * osqp.L_mpc * osqp.B_qp + osqp.K_mpc);

    // std::chrono::system_clock::time_point calTimePmpc = std::chrono::system_clock::now();
    // auto calDiffTimePmpc = std::chrono::duration_cast<std::chrono::microseconds>(calTimePmpc - calTimeXref);
    // std::cout << "Time for calculate P_mpc: " << calDiffTimePmpc.count() << " [us]" << std::endl;

    // osqp.q_mpc = 2. * osqp.B_qp_transpose * osqp.L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);


    // asDiagonal 함수가 먹히는지 잠시 테스트
    // asTemp = osqp.q_mpc.asDiagonal();
    // std::cout << asTemp << std::endl;
    

    //** 알고리즘 2. 불필요한 연산을 하지 않도록 for 루프를 사용하긴 하는데, Eigen3 의 자료형인 MatrixXf 를 그대로 사용하여 P_mpc 와 q_mpc 계산 알고리즘을 수정토록 해보자(불필요한 연산을 없애기 위해서)
    // int start_jj = 0;
    // // int start_k = 0;
    // // int setstart_k = 0;

    // // // MatrixXf B_qp_transpose_time_L_mpc = MatrixXf::Zero(12*horizontal_length, 13*horizontal_length);
    // // // MatrixXf Bt_L_B = MatrixXf::Zero(12*horizontal_length, 12*horizontal_length);

    // for(size_t i = 0; i < horizontal_length * 12; ++i){
    //     for(size_t j = start_jj; j < horizontal_length * 13; ++j){
    //         osqp.B_qp_transpose_time_L_mpc(i, j) = osqp.B_qp_transpose(i, j) * osqp.L_mpc(j, j);
    //     }


    //     if((i) % 12 == 0){
    //         start_jj += 13;
    //     }
    // }
    // start_jj = 0;

    // std::cout << "is here error?" << std::endl;

    // for (unsigned int i = 0; i < horizontal_length * 12; i++) {
    //     for (unsigned int j = start_j; j < horizontal_length * 12; j++) {
    //         osqp.Bt_L_B(i, j) = 0.0;
    //         for (unsigned int k = start_k; k < horizontal_length * 13; k++) {
    //             osqp.Bt_L_B(i, j) += osqp.B_qp_transpose_time_L_mpc(i, k) * osqp.B_qp(k, j);
    //         }
    //         if (i == j)
    //             osqp.P_mpc(i, i) = 2 * (osqp.Bt_L_B(i, i) + osqp.K_mpc(i, i)); // 2*(B_mpc'*Q*B_mpc)+ 2*R
    //         else
    //             osqp.P_mpc(i, j) = 2 * osqp.Bt_L_B(i, j);
    //         if ((j) % 12 == 0) {
    //             start_k += 13;
    //         }
    //     }
    //     start_j++;
    //     start_k = setstart_k;
    //     if ((i) % 12 == 0) {
    //         //            start_j = (i + 1) / 12 * 12;
    //         setstart_k = (i) / 12 * 13;
    //     }
    // }

    // osqp.q_mpc = 2. * osqp.B_qp_transpose_time_L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);

    // std::cout << "is here error2?" << std::endl;

    //** 알고리즘 3. 불필요한 연산을 하지 않도록 for 루프를 사용하긴 하는데, Eigen3 의 자료형인 MatrixXf 를 사용하는 대신 2차원 정적 배열을 미리 선언해보자.
    int start_j = 0;
    int start_k = 0;
    int setstart_k = 0;

    std::chrono::system_clock::time_point calTimeEigen2ArrayStart = std::chrono::system_clock::now();

    float A_qp[13*horizontal_length][13] = {};
    float B_qp[13*horizontal_length][12*horizontal_length] = {};
    float B_qp_transpose[12*horizontal_length][13*horizontal_length] = {};
    float B_qp_transpose_time_L_mpc[12*horizontal_length][13*horizontal_length] = {};
    float Bt_L_B[12*horizontal_length][12*horizontal_length] = {};
    float L_mpc[13*horizontal_length][13*horizontal_length] = {};
    float K_mpc[12*horizontal_length][12*horizontal_length] = {};
    float X_mpc[13];
    float X_ref_horizon[13*horizontal_length];

    // 여기에서 데이터들을 다 복사한다고 가정하자.
    for(size_t i = 0; i < 13*horizontal_length; ++i){
        for(size_t j = 0; j < 13; ++j){
            A_qp[i][j] = osqp.A_qp(i, j);
            
            if(i == 0){
                X_mpc[j] = osqp.X_mpc(j);
            }
        }
        X_ref_horizon[i] = osqp.X_ref_horizon(i);
    }

    for(size_t i = 0; i < 12*horizontal_length; ++i){
        for(size_t j = 0; j < 13*horizontal_length; ++j){
            B_qp_transpose[i][j] = osqp.B_qp_transpose(i, j);
        }
    }

    for(size_t i = 0; i < 13*horizontal_length; ++i){
        for(size_t j = 0; j < 12*horizontal_length; ++j){
            B_qp[i][j] = osqp.B_qp(i, j);
        }
    }

    for(size_t i = 0; i < 13*horizontal_length; ++i){
        L_mpc[i][i] = osqp.L_mpc(i, i);
        if(i < 12*horizontal_length){
            K_mpc[i][i] = osqp.K_mpc(i, i);
        }
    }
    
    //std::cout << "B_qp_Transpose_time_L_mpc(num): " << std::endl;
    for(size_t i = 0; i < horizontal_length * 12; ++i){
        for(size_t j = start_j; j < horizontal_length * 13; ++j){
            B_qp_transpose_time_L_mpc[i][j] = B_qp_transpose[i][j] * L_mpc[j][j];
            //std::cout << B_qp_transpose_time_L_mpc[i][j] << "\t";
        }

        //** 이 알고리즘이 잘못 작동해서 계산 결과가 이상한 것 같다(내가 잘못 넣었을 수도)
        if((i) % 12 == 0 && i != 0){
            start_j += 13;
        }
        //std::cout << std::endl;
    }
    start_j = 0;

    // osqp.B_qp_transpose_time_L_mpc = osqp.B_qp_transpose * osqp.L_mpc;
    // std::cout << "B_qp_Transpose_time_L_mpc(eigen): \n" << osqp.B_qp_transpose_time_L_mpc << std::endl;

    // std::cout << "P_mpc(num): " << std::endl;

    osqp.P_mpc = MatrixXf::Zero(3 * 4 * horizontal_length, 3 * 4 * horizontal_length);
    osqp.q_mpc = VectorXf::Zero(horizontal_length * 12);    

    for (unsigned int i = 0; i < horizontal_length * 12; i++) {
        for (unsigned int j = start_j; j < horizontal_length * 12; j++) {
            Bt_L_B[i][j] = 0.0;
            
            for (unsigned int k = start_k; k < horizontal_length * 13; k++) {
                Bt_L_B[i][j] += B_qp_transpose_time_L_mpc[i][k] * B_qp[k][j];
            }
            if (i == j)
                osqp.P_mpc(i, i) = 2 * (Bt_L_B[i][i] + K_mpc[i][i]); // 2*(B_mpc'*Q*B_mpc)+ 2*R
            else
                osqp.P_mpc(i, j) = 2 * Bt_L_B[i][j];
            

            if ((j) % 12 == 0 && j != 0) {
                start_k += 13;
            }
            //std::cout << "\t" << osqp.P_mpc(i, j) ;
        }
        //std::cout << std::endl;
        start_j++;
        start_k = setstart_k;
        if ((i) % 12 == 0 && i != 0) {
            start_j = i;
            setstart_k = (i) / 12 * 13;
        }
    }

    // std::cout << "P_mpc(eigen): \n" << 2 * (osqp.B_qp.transpose() * osqp.L_mpc * osqp.B_qp + osqp.K_mpc) << std::endl;


    // float A_qp[13*horizontal_length][13] = {};
    // float B_qp[13*horizontal_length][12*horizontal_length] = {};
    // float B_qp_transpose[12*horizontal_length][13*horizontal_length] = {};
    // float B_qp_transpose_time_L_mpc[12*horizontal_length][13*horizontal_length] = {};
    // float Bt_L_B[12*horizontal_length][12*horizontal_length] = {};
    // float L_mpc[13*horizontal_length][13*horizontal_length] = {};
    // float K_mpc[12*horizontal_length][12*horizontal_length] = {};
    // float X_mpc[13];
    // float X_ref_horizon[13*horizontal_length];

    float temp_Ax[13*horizontal_length] = {};
    
    // std::cout << "Ax - y (num): " << std::endl;
    for(size_t i = 0; i < 13*horizontal_length; ++i){
        for(size_t j = 0; j < 13; ++j){
            temp_Ax[i] += A_qp[i][j]*X_mpc[j];
        }
        temp_Ax[i] -= X_ref_horizon[i];
        // std::cout << temp_Ax[i] << std::endl;        
    }

    // std::cout << "Ax - y (eigen): \n" << osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon << std::endl; // Ax-y 는 동일하게 나오는 것 확인하였음

    // std::cout << "q_mpc: " << std::endl;
    
    for(size_t i = 0; i < 12*horizontal_length; ++i){
        for(size_t j = 0; j < 13*horizontal_length; ++j){
            osqp.q_mpc(i) += B_qp_transpose_time_L_mpc[i][j]*temp_Ax[j];
            
        }
        osqp.q_mpc(i) *= 2.;
        // std::cout << osqp.q_mpc(i) << "\n";
        
    }
    // osqp.q_mpc = 2. * osqp.B_qp_transpose_time_L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);
    // osqp.q_mpc = 2. * osqp.B_qp_transpose * osqp.L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);

    // std::cout << "by using eigen, q_mpc: \n" << osqp.q_mpc << std::endl; 
    

    std::chrono::system_clock::time_point calTimeEigen2ArrayEnd = std::chrono::system_clock::now();
    auto calDiffTimeEigen2Array = std::chrono::duration_cast<std::chrono::microseconds>(calTimeEigen2ArrayEnd - calTimeEigen2ArrayStart);
    std::cout << "Time for change MatrixXf to static Array and Calculate P_mpc and q_mpc: " << calDiffTimeEigen2Array.count() << " [us]" << std::endl;

    // // MatrixXf B_qp_transpose_time_L_mpc = MatrixXf::Zero(12*horizontal_length, 13*horizontal_length);
    // // MatrixXf Bt_L_B = MatrixXf::Zero(12*horizontal_length, 12*horizontal_length);

    // for(size_t i = 0; i < horizontal_length * 12; ++i){
    //     for(size_t j = start_j; j < horizontal_length * 13; ++j){
    //         osqp.B_qp_transpose_time_L_mpc(i, j) = osqp.B_qp_transpose(i, j) * osqp.L_mpc(j, j);
    //     }


    //     if((i) % 12 == 0){
    //         start_j += 13;
    //     }
    // }
    // start_j = 0;

    // std::cout << "is here error?" << std::endl;

    // for (unsigned int i = 0; i < horizontal_length * 12; i++) {
    //     for (unsigned int j = start_j; j < horizontal_length * 12; j++) {
    //         osqp.Bt_L_B(i, j) = 0.0;
    //         for (unsigned int k = start_k; k < horizontal_length * 13; k++) {
    //             osqp.Bt_L_B(i, j) += osqp.B_qp_transpose_time_L_mpc(i, k) * osqp.B_qp(k, j);
    //         }
    //         if (i == j)
    //             osqp.P_mpc(i, i) = 2 * (osqp.Bt_L_B(i, i) + osqp.K_mpc(i, i)); // 2*(B_mpc'*Q*B_mpc)+ 2*R
    //         else
    //             osqp.P_mpc(i, j) = 2 * osqp.Bt_L_B(i, j);
    //         if ((j) % 12 == 0) {
    //             start_k += 13;
    //         }
    //     }
    //     start_j++;
    //     start_k = setstart_k;
    //     if ((i) % 12 == 0) {
    //         //            start_j = (i + 1) / 12 * 12;
    //         setstart_k = (i) / 12 * 13;
    //     }
    // }




    // std::chrono::system_clock::time_point calTimePmpcQmpc = std::chrono::system_clock::now();
    // auto calDiffTimeQmpc = std::chrono::duration_cast<std::chrono::microseconds>(calTimePmpcQmpc - calTimePmpc);
    // std::cout << "Time for calculate q_mpc: " << calDiffTimeQmpc.count() << " [us]" << std::endl;

    // auto calDiffTimePmpcQmpc = std::chrono::duration_cast<std::chrono::microseconds>(calTimePmpcQmpc - calTimeXref);   
    // std::cout << "Time for calculate P_mpc and q_mpc: " << calDiffTimePmpcQmpc.count() << " [us]" << std::endl; // 20000 us 이상 소모되는 것 확인

    // std::cout << "osqp.P_mpc: \n" << osqp.P_mpc << std::endl;
    // std::cout << "osqp.q_mpc: \n" << osqp.q_mpc << std::endl;


    // ===================== OSQP  ====================== //
    int row_index = 0;
    int row_index_max = 0;
    int col_index = 0;

    // ===================== P_x ====================== //
    //    c_int P_nnz = 0;
    int OSQP_num1 = 0;
    int OSQP_num2 = 0;
    int OSQP_num3 = 0;

    //    for (int j = 0; j <= osqp.P_mpc.cols() - 1; ++j) {
    //        for (int i = 0; i <= j; ++i) {
    //            if (osqp.P_mpc(i, j) != 0) {
    //                P_nnz++;
    //            }
    //        }
    //    }
    //    c_float OSQP_P_x[P_nnz];
    //    c_int OSQP_P_i[P_nnz];
    //    c_int OSQP_P_p[osqp.P_mpc.cols() + 1];
    // 5. osqp QP form 의 P에 대한 행렬들을 위에서 만든 osqp.P 행렬을 이용하여 만들어줄 것(희소행렬로 표현해주기 위한 성분들을 만들 것)
    // ===================== P_x ====================== //    
    //    osqp.data_P_x[]
    for (int j = 0; j < osqp.P_mpc.cols(); ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i <= j; ++i) {
            if (osqp.P_mpc(i, j) != 0) {
                osqp.P_x_mpc[OSQP_num1] = osqp.P_mpc(i, j);
                osqp.P_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.P_p_mpc[j + 1] = OSQP_num3;
    }
    //    std::cout << "B" << std::endl;
    // ===================== P_i ====================== //
    //osqp.data_P_i[]


    // ===================== P_p ====================== //
    //osqp.data_P_p[]


    // ===================== q ====================== //
    for (unsigned int i = 0; i < horizontal_length * 12; ++i) {
        osqp.q_mpc_[i] = osqp.q_mpc(i);
    }
    //    std::cout << "C" << std::endl;


    //* WsJi - A Matrix prepare
    Vector3f n_(0, 0, 1);
    Vector3f t1_(1, 0, 0);
    Vector3f t2_(0, 1, 0);

    float fric_coef = 0.8;

    MatrixXf C1_(5, 3), C2_(5, 3), C3_(5, 3), C4_(5, 3);
    C1_.block(0, 0, 1, 3) = (-fric_coef * n_ + t1_).transpose();
    C1_.block(1, 0, 1, 3) = (-fric_coef * n_ + t2_).transpose();
    C1_.block(2, 0, 1, 3) = (fric_coef * n_ + t2_).transpose();
    C1_.block(3, 0, 1, 3) = (fric_coef * n_ + t1_).transpose();
    C1_.block(4, 0, 1, 3) = (n_).transpose();

    C2_ = C1_;
    C3_ = C1_;
    C4_ = C1_;

    osqp.C_.block(0, 0, 5, 3) = C1_;
    osqp.C_.block(5, 3, 5, 3) = C2_;
    osqp.C_.block(10, 6, 5, 3) = C3_;
    osqp.C_.block(15, 9, 5, 3) = C4_;


    //** MPC horizontal for osqp.C
    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.C_mpc.block(i * 20, i * 12, 20, 12) = osqp.C_;
    }

    // std::cout << "osqp.C_mpc: \n" << osqp.C_mpc << std::endl;

    //
    /*
    c_int mA_nnz;

    std::vector<double> mA_x;
    std::vector<int> mA_i, mA_p;

    unsigned int mIndex = 0;


    for(size_t j = 0; j < osqp.C_.cols(); ++j){
        bool mTemp = false;
        

        for(size_t i = 0; i < osqp.C_.rows(); ++i){

            //std::cout << C_(i,j) << std::endl;
            if(osqp.C_(i,j) != 0){
                
                mA_x.push_back(osqp.C_(i,j));
                mA_i.push_back(i);
                if(!mTemp){
                    mTemp = true;
                    mA_p.push_back(mIndex);
                }
                ++mIndex;
            }
        }
        mTemp = false;
    }    

    mA_nnz = mA_x.size();
    mA_p.push_back(mA_nnz);

    osqp.data_A_nnz = mA_nnz;
     */

    //    c_int A_nnz = 0;
    //    for (int i = 0; i < 20 * horizontal_length; ++i) {
    //        for (int j = 0; j < 12 * horizontal_length; ++j) {
    //            if (osqp.C_mpc(i, j) != 0) {
    //                A_nnz++;
    //            }
    //        }
    //    }

    //    c_float OSQP_A_x[A_nnz];
    //    c_int OSQP_A_i[A_nnz];
    //        std::cout << "P_nnz = " << P_nnz << std::endl;
    //        std::cout << "A_nnz = " << A_nnz << std::endl;
    OSQP_num1 = 0;
    OSQP_num2 = 0;
    OSQP_num3 = 0;
    for (int j = 0; j < 12 * horizontal_length; ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i < 20 * horizontal_length; ++i) {
            if (osqp.C_mpc(i, j) != 0) {
                osqp.A_x_mpc[OSQP_num1] = osqp.C_mpc(i, j);
                osqp.A_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.A_p_mpc[j + 1] = OSQP_num3;
        //        std::cout << "OSQP_A_p[" << j+1<< "]=" <<osqp.A_p[j + 1] << std::endl;
    }
    //        std::cout << "D" << std::endl;
    //////////////

    
    
    // 구속조건인데 얘는 연산시간에 critical 한 영향을 미치지는 않는다.
    for (int i = 0; i < horizontal_length; ++i) {

        // RL
        osqp.data_l_mpc[20 * i] = -10000.;
        osqp.data_l_mpc[20 * i + 1] = -10000.;
        osqp.data_l_mpc[20 * i + 2] = 0.;
        osqp.data_l_mpc[20 * i + 3] = 0.;
        osqp.data_l_mpc[20 * i + 4] = 10.;
        osqp.data_u_mpc[20 * i] = 0.;
        osqp.data_u_mpc[20 * i + 1] = 0.;
        osqp.data_u_mpc[20 * i + 2] = 10000.;
        osqp.data_u_mpc[20 * i + 3] = 10000.;
        osqp.data_u_mpc[20 * i + 4] = 400.;

        // RR
        osqp.data_l_mpc[20 * i + 5] = -10000.;
        osqp.data_l_mpc[20 * i + 6] = -10000.;
        osqp.data_l_mpc[20 * i + 7] = 0.;
        osqp.data_l_mpc[20 * i + 8] = 0.;
        osqp.data_l_mpc[20 * i + 9] = 10.;
        osqp.data_u_mpc[20 * i + 5] = 0.;
        osqp.data_u_mpc[20 * i + 6] = 0.;
        osqp.data_u_mpc[20 * i + 7] = 10000.;
        osqp.data_u_mpc[20 * i + 8] = 10000.;
        osqp.data_u_mpc[20 * i + 9] = 400.;

        // FR
        osqp.data_l_mpc[20 * i + 15] = -10000.;
        osqp.data_l_mpc[20 * i + 16] = -10000.;
        osqp.data_l_mpc[20 * i + 17] = 0.;
        osqp.data_l_mpc[20 * i + 18] = 0.;
        osqp.data_l_mpc[20 * i + 19] = 10.;
        osqp.data_u_mpc[20 * i + 15] = 0.;
        osqp.data_u_mpc[20 * i + 16] = 0.;
        osqp.data_u_mpc[20 * i + 17] = 10000.;
        osqp.data_u_mpc[20 * i + 18] = 10000.;
        osqp.data_u_mpc[20 * i + 19] = 400.;

        // FL
        osqp.data_l_mpc[20 * i + 10] = -10000.;
        osqp.data_l_mpc[20 * i + 11] = -10000.;
        osqp.data_l_mpc[20 * i + 12] = 0.;
        osqp.data_l_mpc[20 * i + 13] = 0.;
        osqp.data_l_mpc[20 * i + 14] = 10.;
        osqp.data_u_mpc[20 * i + 10] = 0.;
        osqp.data_u_mpc[20 * i + 11] = 0.;
        osqp.data_u_mpc[20 * i + 12] = 10000.;
        osqp.data_u_mpc[20 * i + 13] = 10000.;
        osqp.data_u_mpc[20 * i + 14] = 400.;

        if(walking.phase != TROT_NONE){
            // FR
            if(temp_contact_foots(3, i*int(mpc_sampletime/tasktime)) == false){
                osqp.data_l_mpc[20 * i + 15] = -0.;
                osqp.data_l_mpc[20 * i + 16] = -0.;
                osqp.data_l_mpc[20 * i + 17] = 0.;
                osqp.data_l_mpc[20 * i + 18] = 0.;
                osqp.data_l_mpc[20 * i + 19] = 0.;
                osqp.data_u_mpc[20 * i + 15] = 0.;
                osqp.data_u_mpc[20 * i + 16] = 0.;
                osqp.data_u_mpc[20 * i + 17] = 0.;
                osqp.data_u_mpc[20 * i + 18] = 0.;
                osqp.data_u_mpc[20 * i + 19] = 0.;
            }

            // FL
            if(temp_contact_foots(2, i*int(mpc_sampletime/tasktime)) == false){
                osqp.data_l_mpc[20 * i + 10] = -0.;
                osqp.data_l_mpc[20 * i + 11] = -0.;
                osqp.data_l_mpc[20 * i + 12] = 0.;
                osqp.data_l_mpc[20 * i + 13] = 0.;
                osqp.data_l_mpc[20 * i + 14] = 0.;
                osqp.data_u_mpc[20 * i + 10] = 0.;
                osqp.data_u_mpc[20 * i + 11] = 0.;
                osqp.data_u_mpc[20 * i + 12] = 0.;
                osqp.data_u_mpc[20 * i + 13] = 0.;
                osqp.data_u_mpc[20 * i + 14] = 0.;
            }

            // RR
            if(temp_contact_foots(1, i*int(mpc_sampletime/tasktime)) == false){
                osqp.data_l_mpc[20 * i + 5] = -0.;
                osqp.data_l_mpc[20 * i + 6] = -0.;
                osqp.data_l_mpc[20 * i + 7] = 0.;
                osqp.data_l_mpc[20 * i + 8] = 0.;
                osqp.data_l_mpc[20 * i + 9] = 0.;
                osqp.data_u_mpc[20 * i + 5] = 0.;
                osqp.data_u_mpc[20 * i + 6] = 0.;
                osqp.data_u_mpc[20 * i + 7] = 0.;
                osqp.data_u_mpc[20 * i + 8] = 0.;
                osqp.data_u_mpc[20 * i + 9] = 0.;
            }

            // RL
            if(temp_contact_foots(0, i*int(mpc_sampletime/tasktime)) == false){
                osqp.data_l_mpc[20 * i] = -0.;
                osqp.data_l_mpc[20 * i + 1] = -0.;
                osqp.data_l_mpc[20 * i + 2] = 0.;
                osqp.data_l_mpc[20 * i + 3] = 0.;
                osqp.data_l_mpc[20 * i + 4] = 0.;
                osqp.data_u_mpc[20 * i] = 0.;
                osqp.data_u_mpc[20 * i + 1] = 0.;
                osqp.data_u_mpc[20 * i + 2] = 0.;
                osqp.data_u_mpc[20 * i + 3] = 0.;
                osqp.data_u_mpc[20 * i + 4] = 0.;
            }

        }
    }


    //std::cout << "walking.phase: " << walking.phase << std::endl;

    // if (OSQP_data) {
    //     OSQP_data->n = 12 * horizontal_length;
    //     OSQP_data->m = 20 * horizontal_length;
    //     OSQP_data->P = csc_matrix(OSQP_data->n, OSQP_data->n, osqp.P_nnz_mpc, osqp.P_x_mpc, osqp.P_i_mpc, osqp.P_p_mpc);
    //     OSQP_data->q = osqp.q_mpc_;
    //     OSQP_data->A = csc_matrix(OSQP_data->m, OSQP_data->n, osqp.A_nnz_mpc, osqp.A_x_mpc, osqp.A_i_mpc, osqp.A_p_mpc);
    //     OSQP_data->l = osqp.data_l_mpc;
    //     OSQP_data->u = osqp.data_u_mpc;
    // }





    // std::cout << "A_c: \n" << osqp.A_c << std::endl;
    // std::cout << "A_qp: \n" << osqp.A_qp << std::endl;

    // std::cout << "B_c: \n" << osqp.B_c << std::endl;
    // std::cout << "B_qp: \n" << osqp.B_qp << std::endl;

    // std::chrono::system_clock::time_point calMidTime = std::chrono::system_clock::now();
    // auto calDiffTime1 = std::chrono::duration_cast<std::chrono::microseconds>(calMidTime - calStartTime);
    // //calEndTime = std::chrono::system_clock::now();
    // //std::cout << "calStartTime: " << calStartTime << "\ncalEndTime: " << calEndTime << "\ndifference: " << calEndTime - calStartTime << std::endl;
    // std::cout << "Time for calculate MPC algorithm: " << calDiffTime1.count() << " [us]" << std::endl;

    //
    osqp_update_P(OSQP_work, osqp.P_x_mpc, OSQP_NULL, osqp.P_nnz_mpc);
    osqp_update_lin_cost(OSQP_work, osqp.q_mpc_);
    osqp_update_bounds(OSQP_work, osqp.data_l_mpc, osqp.data_u_mpc);

    // Solve Problem
    //std::cout << "I'm alive! (4)" << std::endl;
    osqp_solve(OSQP_work);

    
    // std::chrono::system_clock::time_point calEndTime = std::chrono::system_clock::now();
    // auto calDiffTime2 = std::chrono::duration_cast<std::chrono::microseconds>(calEndTime - calMidTime);
    // std::cout << "Time for OSQP Solver: " << calDiffTime2.count() << " [us]\n" << std::endl;

    //**
    // std::cout << "walking.phase: " << walking.phase << std::endl;
    // std::cout << "osqp.r_fl_vector: \n" << osqp.r_fl_vector << std::endl;
    // std::cout << "osqp.X_mpc: \n" << osqp.X_mpc << std::endl;
    // std::cout << "osqp.X_ref: \n" << osqp.X_ref_horizon << std::endl;
    // std::cout << "osqp.A_c: \n" << osqp.A_c << std::endl;
    // std::cout << "osqp.B_c: \n" << osqp.B_c << std::endl;
    // std::cout << "osqp.A_hat: \n" << osqp.A_hat << std::endl;
    // std::cout << "osqp.B_hat: \n" << osqp.B_hat << std::endl;
    // std::cout << "osqp.A_qp: \n" << osqp.A_qp << std::endl;
    // std::cout << "osqp.B_qp: \n" << osqp.B_qp << std::endl;
    // std::cout << "osqp.P_mpc: \n" << osqp.P_mpc << std::endl;
    // std::cout << "osqp.q_mpc: \n" << osqp.q_mpc << std::endl;
    // std::cout << "osqp.K_mpc: \n" << osqp.K_mpc << std::endl;
    // std::cout << "osqp.L_mpc: \n" << osqp.L_mpc << std::endl;
    // std::cout << "contact point vector: " << contact.Foot(0) << ", " << contact.Foot(1) << ", " << contact.Foot(2) << ", " << contact.Foot(3) << std::endl;

    // std::cout << "Ax-y: \n" << osqp.A_qp*osqp.X_mpc - osqp.X_ref_horizon << std::endl;

    for (unsigned int i = 0; i < 12; ++i) {        
        osqp.Ref_GRF(i) = OSQP_work->solution->x[i];

        // std::cout << "osqp.Ref_GRF(" << i << "): " << osqp.Ref_GRF(i) << std::endl;

        // if(abs(osqp.Ref_GRF(i)) > 10000.){
        //     osqp.Ref_GRF = VectorXf::Zero(12);
        //     break;
        // }
    }

    //**
    
    ROT_WORLD2BASE                              = EulerZYX2RotMat(base.G_CurrentOri);
    ROT_BASE2WORLD                              = ROT_WORLD2BASE.transpose();

    for (int i = 0; i < 4; i++) {
        ROT_WORLD2BASE_TOTAL.block(i*3, i*3, 3, 3) = ROT_WORLD2BASE;
        ROT_BASE2WORLD_TOTAL.block(i*3, i*3, 3, 3) = ROT_BASE2WORLD;
    }
    
    // osqp.Jacobian.block( 0,0,6,18)           = J_BASE;
    osqp.Jacobian.block( 0,0,3,18)              = J_FL.cast <float> ();
    osqp.Jacobian.block( 3,0,3,18)              = J_FR.cast <float> ();
    osqp.Jacobian.block( 6,0,3,18)              = J_RL.cast <float> ();
    osqp.Jacobian.block( 9,0,3,18)              = J_RR.cast <float> ();



    
    osqp.tmp_torque                             = -osqp.Select * osqp.Jacobian.transpose() * ROT_BASE2WORLD_TOTAL * (osqp.Ref_GRF);
    // osqp.tmp_torque                             = osqp.Select * osqp.Jacobian.transpose() * ROT_BASE2WORLD_TOTAL * (osqp.Ref_GRF);
    
    for (unsigned int i = 0; i < 12; ++i) {
        osqp.torque(i + BaseDOF)                = osqp.tmp_torque(i);
    }

    // std::cout << "osqp.torque: \n" << osqp.torque << std::endl;
    //std::cout << "bodyvel: \n" << base.G_GroundTruthVel << std::endl;
    //** 토크가 잘못돼도 단단히 잘못된듯 하다.... 억지로 인가를 해보자.
    // osqp.torque(0+BaseDOF) = -16.8;
    // osqp.torque(1+BaseDOF) = 22.1;
    // osqp.torque(2+BaseDOF) = 31.;

    // osqp.torque(3+BaseDOF) = 16.8;
    // osqp.torque(4+BaseDOF) = 22.1;
    // osqp.torque(5+BaseDOF) = 31.;

    // osqp.torque(6+BaseDOF) = -17.805;
    // osqp.torque(7+BaseDOF) = -16.885;
    // osqp.torque(8+BaseDOF) = 15.31;

    // osqp.torque(9+BaseDOF) = 17.805;
    // osqp.torque(10+BaseDOF) = -16.885;
    // osqp.torque(11+BaseDOF) = 15.31;
        
}

void CRobot::TorqueOff(void) {
   /* TorqueOff
    * All joint Torque are set to Zero
    */
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
        joint[nJoint].RefTorque                    = 0;
        
        joint[nJoint].RefPos                    = 0;
        joint[nJoint].RefVel                    = 0;
        joint[nJoint].RefAcc                    = 0;
    }                
                
    walking.Stop                                = false;
    walking.MoveDone                            = false;
    walking.Ready                               = false;
    torque.CTC_Flag                             = false;
    osqp.Flag                                   = false;
    friction.Activeflag                         = false;
                
    CommandFlag                                 = NONE_ACT;
}

void CRobot::HomePose(float readytime) {
   /* HomePose
    * Move at the set Joint Angle
    */
    const float KpGain                         = 600;
    const float KdGain                         = 6;
    static float time                          = 0;
    static bool flag                           = 0;

    static VectorXf JointRefPos                = VectorXf::Zero(12);
    static VectorXf JointRefPrePos             = VectorXf::Zero(12);
    static VectorXf JointRefVel                = VectorXf::Zero(12);

    if(walking.Ready == true){
        return;
    }
    
    if(time == 0) {
        cout << "[PongBot] : Home Pose" << endl;
        for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
            joint[nJoint].InitPos                           = joint[nJoint].CurrentPos;
            JointRefPos[nJoint]                             = joint[nJoint].InitPos;
            JointRefPrePos[nJoint]                          = joint[nJoint].InitPos;
        }
    }          
        
    if(time <= readytime) {
        walking.time.sec                                    = time;
       
        for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
            JointRefPos[nJoint]                             = cosWave(joint[nJoint].RefTargetPos - joint[nJoint].InitPos, readytime, walking.time.sec, joint[nJoint].InitPos);
            JointRefVel[nJoint]                             = (JointRefPos[nJoint] - JointRefPrePos[nJoint])/tasktime;
            
            joint[nJoint].RefTorque                         = KpGain*(JointRefPos[nJoint] - joint[nJoint].CurrentPos) + KdGain*(JointRefVel[nJoint] - joint[nJoint].CurrentVel);
            
            JointRefPrePos[nJoint]                          = JointRefPos[nJoint];
        }
        time += tasktime;
    }
    else if(time < 2.5*readytime){
        for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
            joint[nJoint].RefTorque                         = KpGain*(JointRefPos[nJoint] - joint[nJoint].CurrentPos) + KdGain*(JointRefVel[nJoint] - joint[nJoint].CurrentVel);
        }
        time += tasktime;
    }        
    else{
        if(flag == 0){
            for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
                joint[nJoint].Incremental_CurrentPosOffset      = joint[nJoint].CurrentPos - joint[nJoint].Incremental_CurrentPos;
            }
            IMUNulling();
            flag = true;
        }
        
        for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
            joint[nJoint].RefTorque                         = KpGain*(JointRefPos[nJoint] - joint[nJoint].CurrentPos) + KdGain*(JointRefVel[nJoint] - joint[nJoint].CurrentVel);
        }
        
        walking.HomePoseDone                    = true;
        ControlMode                             = CTRLMODE_WALK_READY;
        CommandFlag                             = WALK_READY;
    }
}

void CRobot::GetFrictionCoefficient(float readytime) {
    static double time                  = 0;
    
    static double TargetCurrent         = 2.0;
    static double Period                = 2.0;

    if(time == 0){
        cout << "Get Friction Coefficient Start" << endl;
    }
    if(time <= readytime) {
        walking.time.sec               = time;
        friction.Current[FLHR]         = TargetCurrent * sin((2*PI*walking.time.sec)/Period);
        
        time += tasktime;
    }
    else{
        cout << "Get Friction Coefficient Complete" << endl;
    }
}

void CRobot::WalkingReady(float readytime) {
    /* WalkingReady
     * input : readytime
     * output : Set CoM & Foot Position for WalkingReady Motion
     */
    static float time                          = 0;
    
    if(walking.Ready == true) {
        return;
    }
    if(time == 0) {  
        cout << "[PongBot] : Walking Ready Start" << endl;
        torque.CTC_Flag                         = true;
        osqp.Flag                               = true;
                         
        // Set Previous Position                
        com.G_RefPrePos                              = com.G_RefPos;
        com.G_RefPreVel                              = com.G_RefVel;
        com.G_RefPreAcc                              = com.G_RefAcc;
                         
        base.G_RefPreOri                             = base.G_RefOri;
        base.G_RefPreAngularVel                      = base.G_RefAngularVel;
        base.G_RefPreAngularAcc                      = base.G_RefAngularAcc;

        RL_Foot.G_RefPrePos                          = RL_Foot.G_RefPos;
        RR_Foot.G_RefPrePos                          = RR_Foot.G_RefPos;
        FL_Foot.G_RefPrePos                          = FL_Foot.G_RefPos;
        FR_Foot.G_RefPrePos                          = FR_Foot.G_RefPos;
    }
    
    if(time < readytime) {
        walking.time.sec                             = time;
        com.G_RefPos                                 = cosWave(com.G_RefTargetPos - com.G_RefPrePos, readytime, walking.time.sec, com.G_RefPrePos);
        com.G_RefVel                                 = differential_cosWave(com.G_RefTargetPos - com.G_RefPrePos, readytime, walking.time.sec);
        
        FL_Foot.G_RefPos                            = cosWave(FL_Foot.G_RefTargetPos - FL_Foot.G_RefPrePos, readytime, walking.time.sec, FL_Foot.G_RefPrePos);
        FR_Foot.G_RefPos                            = cosWave(FR_Foot.G_RefTargetPos - FR_Foot.G_RefPrePos, readytime, walking.time.sec, FR_Foot.G_RefPrePos);
        RL_Foot.G_RefPos                            = cosWave(RL_Foot.G_RefTargetPos - RL_Foot.G_RefPrePos, readytime, walking.time.sec, RL_Foot.G_RefPrePos);
        RR_Foot.G_RefPos                            = cosWave(RR_Foot.G_RefTargetPos - RR_Foot.G_RefPrePos, readytime, walking.time.sec, RR_Foot.G_RefPrePos);
                
        FL_Foot.G_RefVel                            = differential_cosWave(FL_Foot.G_RefTargetPos - FL_Foot.G_RefPrePos, readytime, walking.time.sec);
        FR_Foot.G_RefVel                            = differential_cosWave(FR_Foot.G_RefTargetPos - FR_Foot.G_RefPrePos, readytime, walking.time.sec);        
        RL_Foot.G_RefVel                            = differential_cosWave(RL_Foot.G_RefTargetPos - RL_Foot.G_RefPrePos, readytime, walking.time.sec);
        RR_Foot.G_RefVel                            = differential_cosWave(RR_Foot.G_RefTargetPos - RR_Foot.G_RefPrePos, readytime, walking.time.sec);

        time += tasktime;
    }
    else
    {
        time = 0.0;
        walking.time.sec                        = time;
                
        com.G_RefPos                              = com.G_RefTargetPos;
        com.G_RefVel                              = com.G_RefTargetVel;
                
        base.G_RefOri                             = base.G_RefTargetOri;
        base.G_RefAngularVel                      = base.G_RefTargetAngularVel;
                
        FL_Foot.G_RefPos                          = FL_Foot.G_RefTargetPos;
        FR_Foot.G_RefPos                          = FR_Foot.G_RefTargetPos;
        RL_Foot.G_RefPos                          = RL_Foot.G_RefTargetPos;
        RR_Foot.G_RefPos                          = RR_Foot.G_RefTargetPos;
                
        FL_Foot.G_RefVel                          = FL_Foot.G_RefTargetVel;
        FR_Foot.G_RefVel                          = FR_Foot.G_RefTargetVel;        
        RL_Foot.G_RefVel                          = RL_Foot.G_RefTargetVel;
        RR_Foot.G_RefVel                          = RR_Foot.G_RefTargetVel;
                        
        FL_Foot.G_RefPrePos                          = FL_Foot.G_RefPos;
        FR_Foot.G_RefPrePos                          = FR_Foot.G_RefPos;
        RL_Foot.G_RefPrePos                          = RL_Foot.G_RefPos;
        RR_Foot.G_RefPrePos                          = RR_Foot.G_RefPos;
                
        FL_Foot.G_RefPreVel                          = FL_Foot.G_RefVel;
        FR_Foot.G_RefPreVel                          = FR_Foot.G_RefVel;        
        RL_Foot.G_RefPreVel                          = RL_Foot.G_RefVel;
        RR_Foot.G_RefPreVel                          = RR_Foot.G_RefVel;
                
        walking.Ready                           = true;
        walking.MoveDone                        = true;
        slope.Flag                              = true;
        CommandFlag                             = NONE_ACT;
        cout << "[PongBot] : Walking Ready Complete" << endl;
    }
}

void CRobot::Stand(void) {
    static float DegreeRange                    = 15.0;
    static float DegreeRange_Yaw                = 10.0;
    static float PositionRange                  = 0.1;
     
    static float ActivePositionThreshold        = 0.0001;
    static float ActiveDegreeThreshold          = 0.0001;
     
    static float EulerZYX_dotLimit              = 15*D2R;   //[rad/s]
    static float VelLimit                       = 0.1;      //[m/s]

    static float Time                           = tasktime; // [s]
    static int ActiveNumber;
    
    //For Calculation
    float** tempOutMat                          = matrix(1, 3, 1, 3);
    float* OutVec                               = fvector(1, 3);
    float* tempOutVec                           = fvector(1, 3);
    
    if ((walking.MoveDone == false)||(walking.Ready == false)){
        return;
    }
    
    if((standturn.Startflag == true)&&(walking.UpDown == true)){
        cout << "[PongBot] : Stand Mode" << endl;
        standturn.G_InitOri                                 = base.G_RefOri;
        standturn.G_RefPreOri                               = base.G_RefOri;
        standturn.G_RefTargetOri                            = standturn.G_InitOri;

        standturn.G_InitPos                                 = com.G_RefPos;
        standturn.G_RefPrePos                               = com.G_RefPos;
        standturn.G_RefTargetPos                            = standturn.G_InitPos;
        
        standturn.Startflag                                 = false;

    }
    else if(standturn.Startflag == false){
        ActiveNumber                                        = 0;
        
        // Orientation
        standturn.G_RefTargetOri(AXIS_ROLL)                 = (standturn.G_InitOri(AXIS_ROLL)     + walking.InputData[AXIS_Y]   * DegreeRange * D2R);
        standturn.G_RefTargetOri(AXIS_PITCH)                = (standturn.G_InitOri(AXIS_PITCH)    + walking.InputData[AXIS_X]   * DegreeRange * D2R);
        standturn.G_RefTargetOri(AXIS_YAW)                  = (standturn.G_InitOri(AXIS_YAW)      + walking.InputData[AXIS_YAW] * DegreeRange_Yaw * D2R);
                
        standturn.G_RefOri_dot(AXIS_ROLL)                   = (standturn.G_RefTargetOri(AXIS_ROLL)   - standturn.G_RefPreOri(AXIS_ROLL))/Time;
        standturn.G_RefOri_dot(AXIS_PITCH)                  = (standturn.G_RefTargetOri(AXIS_PITCH)  - standturn.G_RefPreOri(AXIS_PITCH))/Time;
        standturn.G_RefOri_dot(AXIS_YAW)                    = (standturn.G_RefTargetOri(AXIS_YAW)    - standturn.G_RefPreOri(AXIS_YAW))/Time;
 
        for (int AXIS = 0; AXIS < 3; AXIS++){
            if(standturn.G_RefOri_dot(AXIS) > EulerZYX_dotLimit){
                standturn.G_RefOri_dot(AXIS) = EulerZYX_dotLimit;
            }
            else if(standturn.G_RefOri_dot(AXIS) < -EulerZYX_dotLimit){
                standturn.G_RefOri_dot(AXIS) = -EulerZYX_dotLimit;
            }
        }
        
        for (int AXIS = 0; AXIS < 3; AXIS++){
            base.G_RefOri(AXIS)                             = standturn.G_RefPreOri(AXIS)  + standturn.G_RefOri_dot(AXIS)*Time;
        }
        
        
        //* Calculate E_ZYX
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            tempOutVec[AXIS + 1]                            = standturn.G_RefOri_dot(AXIS);
        }
        setMappingE_ZYX(base.G_RefOri, tempOutMat); 
        mvmult(tempOutMat, 3, 3, tempOutVec, 3, OutVec);  
        
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            base.G_RefAngularVel(AXIS)                      = OutVec[AXIS + 1];
        }

        standturn.G_RefPreOri                               = base.G_RefOri;

        // Position
        standturn.G_RefTargetPos(AXIS_Z)                    = (standturn.G_InitPos(AXIS_Z)      + 0.5 * (walking.CoM_InputData[AXIS_Z] - 1) * PositionRange);
        standturn.G_RefVel(AXIS_Z)                          = (standturn.G_RefTargetPos(AXIS_Z)  - standturn.G_RefPrePos(AXIS_Z))/Time;

        if(standturn.G_RefVel(AXIS_Z) > VelLimit){
            standturn.G_RefVel(AXIS_Z) = VelLimit;
        }
        else if(standturn.G_RefVel(AXIS_Z) < -VelLimit){
            standturn.G_RefVel(AXIS_Z) = -VelLimit;
        }        
        
        com.G_RefVel(AXIS_Z)                                = standturn.G_RefVel(AXIS_Z);
        com.G_RefPos(AXIS_Z)                                = standturn.G_RefPrePos(AXIS_Z)  + standturn.G_RefVel(AXIS_Z) *Time;
        
        standturn.G_RefPrePos(AXIS_Z)                            = com.G_RefPos(AXIS_Z);

        for (int AXIS = 0; AXIS < 3; AXIS++){
            if (abs((standturn.G_InitPos(AXIS) - com.G_RefPos(AXIS)))            < ActivePositionThreshold){
                standturn.Activeflag_XYZ(AXIS) = true;
            }
            else{
                standturn.Activeflag_XYZ(AXIS) = false;
            }
        }
        
        for (int AXIS = 0; AXIS < 3; AXIS++){
            if (abs((standturn.G_InitOri(AXIS) - base.G_RefOri(AXIS)))           < ActiveDegreeThreshold * D2R){
                standturn.Activeflag_RPY(AXIS) = true;
            }
            else{
                standturn.Activeflag_RPY(AXIS) = false;
            }
        }

        for (int AXIS = 0; AXIS < 3; AXIS++){
            ActiveNumber = ActiveNumber + standturn.Activeflag_XYZ(AXIS) + standturn.Activeflag_RPY(AXIS);
        }
        
        if(ActiveNumber == 6){
            walking.UpDown = true;
        }
        else{
            walking.UpDown = false;
        }
    }
    
    free_matrix(tempOutMat, 1, 3, 1, 3);
    free_vector(OutVec, 1, 3);
    free_vector(tempOutVec, 1, 3);
}
#if 0
void CRobot::Trot(void) {
    /* Trot
     * Set CoM & Foot Position for Trot Motion
     */
    static float com_motion_time;
    static float foot_motion_time;
            
    if (walking.Ready == false){
        return;
    }
    
    if (walking.UpDown == false){
        cout << "[PongBot] : Trot Mode can't activate ! " << endl;
        ControlMode                                     = CTRLMODE_WALK_READY;
        CommandFlag                                     = STAND;
        return;
    }
    
    if(walking.MoveDone == true){
        walking.MoveDone                                = false;
        walking.step.count                              = 0;
        cout << "[PongBot] : Trot Mode" << endl;
    }
    
    if (walking.step.count == 0) {
        Trot_init();
        walking.phase                                   = TROT_READY;
    }
    
    if (walking.step.count == walking.step.RLFR_SWING_START + onesecScale*tasktime) {
        
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //** 추가된 부분 조심
        //** 미래 궤적을 설계할 때, start_foot 이 -1, 0, 1 만으로는 표현하기 어려움을 느낌.
        //** 0일때 이전이 -1이었는지 1이었는지 표현해주는 것이 필요
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;
    }
    else if (walking.step.count == walking.step.RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
                        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;


        //** 추가된 부분 조심
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;


        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                             
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;


        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_END + onesecScale*tasktime) {
        CommandFlag                                     = NONE_ACT;
        walking.MoveDone                                = true;
    }
    
    switch(walking.phase){
        case TROT_SWING:

            if (walking.step.count <= walking.step.RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);
            }
            else if (walking.step.count <= walking.step.FINAL_RLFR_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);
            }
            else if (walking.step.count <= walking.step.FINAL_RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
            }                      
            else {                      
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
            }
            GainSchedule(walking.step.start_foot, foot_motion_time, walking.time.swing);

            generateCoMTraj(com_motion_time);    
            generateFootTraj_Trot(foot_motion_time, walking.step.start_foot);
            
            break;

        case TROT_STANCE:
            
            if (walking.step.count <= walking.step.RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);   
            }
            else if (walking.step.count <= walking.step.FINAL_RLFR_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);

//                if ((walking.step.count == walking.step.FINAL_RLFR_SWING_START -onesecScale*tasktime)&&(walking.Stop == false)){
                if ((walking.step.count == walking.step.FINAL_RLFR_SWING_START)&&(walking.Stop == false)){
//                    walking.step.count                  = walking.step.RLFR_SWING_START-onesecScale*tasktime;
                    walking.step.count                  = walking.step.RLFR_SWING_START;

                }
            }
            else if (walking.step.count <= walking.step.FINAL_RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
            }            
            else {            
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
            }

            
            
            GainSchedule(walking.step.start_foot, com_motion_time, walking.time.stance);   
            generateCoMTraj(com_motion_time);
            break;

        case TROT_NONE: 
            break;
    }
    
    if (walking.MoveDone == false){
        walking.step.count += onesecScale*tasktime;
    }


    //** 미래의 궤적들을 Trot 내부에서 만드는 게 낫겠다고 생각하게 되었다.
    //** 이유: 현재는 특정 버튼을 누르면 트롯이 동작하기 때문에, 다른 함수에서 트롯을 또 동작시키면 com_motion_time 등이 계속 커지는 골치가 생긴다.
    //** Trot 이 한번 호출될 때 미래 궤적까지 계산되도록 하는 게 바람직하다고 판단하였으며, 다리와 몸체의 미래 궤적들을 저장하기 위한 전역 변수를 새로 만들 필요가 생겼다.
    //VectorXf temp_com_ref = VectorXf::Zero(3); // 전역으로 만들기 전에 임시로

    //** ref 에 영벡터 대신 덮어씌우는 용도
    // for(size_t i = 0; i < 100; ++i){ // 100 은 미래를 바라보려는 갯수. 추후 Trot 함수의 디폴트 매개변수를 따로 만들어주어야 할 것이다.
    //     //temp_com_ref.block<3, 1>(i*3, 0) = generateCoMTraj_modified(com_motion_time, i+1);
    //     // temp_FR_ref.block<3, 1>(i*3, 0) = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, i+1);
    //     temp_legs_ref.block<3, 1>(i*3, 0) = FL_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 1) = FR_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 2) = RL_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 3) = RR_Foot.G_RefPos;
    // }

    temp_com_ref.block<3, 1>(0, 0) = com.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 0) = FL_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 1) = FR_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 2) = RL_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 3) = RR_Foot.G_RefPos;

    // std::cout << com.G_RefPos << std::endl;
    // std::cout << FL_Foot.G_RefPos << std::endl;
    temp_com2FL_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 0) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2FR_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 1) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2RL_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 2) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2RR_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 3) - temp_com_ref.block<3, 1>(0, 0);

    for(size_t i = 1; i < 100; ++i){ // 100 은 미래를 바라보려는 갯수. 추후 Trot 함수의 디폴트 매개변수를 따로 만들어주어야 할 것이다.
        temp_com_ref.block<3, 1>(i*3, 0) = generateCoMTraj_modified(com_motion_time, i+1);
        // temp_FR_ref.block<3, 1>(i*3, 0) = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, i+1);
        temp_legs_ref.block<3, 4>(i*3, 0) = generateFootTraj_Trot_modified(com_motion_time, foot_motion_time, walking.step.start_foot, walking.step.start_foot_prev, i+1);


        temp_com2FL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2FR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 1) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2RL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 2) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2RR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 3) - temp_com_ref.block<3, 1>(i*3, 0);
    }

    //VectorXf temp_com2FL_ref = VectorXf::Zero(3*100);
    //std::cout << "temp:com2FL_ref: \n" << temp_com2FL_ref << std::endl;



    // temp_FR_ref = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, 1);
    //temp_FR_ref2 = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, 2);


    // plotjuggler 에서 찍어보기 위해, static 변수들을 잠시 전역 변수에 복사
    temp_com_motion_time = com_motion_time;
    temp_foot_motion_time = foot_motion_time;
    // std::cout << walking.step.count << std::endl;
    
}
#endif
void CRobot::Trot(void) {
    /* Trot
     * Set CoM & Foot Position for Trot Motion
     */
    static float com_motion_time;
    static float foot_motion_time;

    // 1. Flag 를 검사하고 있음            
    if (walking.Ready == false){
        return;
    }
    
    if (walking.UpDown == false){
        cout << "[PongBot] : Trot Mode can't activate ! " << endl;
        ControlMode                                     = CTRLMODE_WALK_READY;
        CommandFlag                                     = STAND;
        return;
    }
    
    if(walking.MoveDone == true){
        walking.MoveDone                                = false;
        walking.step.count                              = 0;
        cout << "[PongBot] : Trot Mode" << endl;
    }
    
    // 2. count 를 검사, 0  일 시 Trot initialize 를 해준 후 phase 플래그를 바꾼다
    if (walking.step.count == 0) {
        Trot_init();
        walking.phase                                   = TROT_READY;
    }
    

    // 3. walking.step.count 가 특정 시간일 때 마다 walking.phase 가 변경돼고
    // 보아하니 walking.step.count 는 720(0) 부터 720 까지 반복
    // 720 에서 다시 초기화돼는 이유는, TROT_STANCE 에서 walking.stop 이 FALSE 이기 때문에, 여기서 0으로 초기화시켜준다.
    // 만약 사용자가 동그라미 버튼을 통해 trot stop 을 실행할 경우 walking.stop 이 활성화돼며, 카운트는 720~1440 에 도달하게 된다.
    if (walking.step.count == walking.step.RLFR_SWING_START + onesecScale*tasktime) {
        
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
        
        //setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        setWalkingStep();
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
        
        //** 미래 궤적을 설계할 때, start_foot 이 -1, 0, 1 만으로는 표현하기 어려움을 느낌.
        //** 0일때 이전이 -1이었는지 1이었는지 표현해주는 것이 필요
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;


    }
    else if (walking.step.count == walking.step.RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
                        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                             
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_END + onesecScale*tasktime) {
        CommandFlag                                     = NONE_ACT;
        walking.MoveDone                                = true;
        std::cout << "come here??? count: " << walking.step.count << std::endl;
    }
    
    // 4. 현재 페이즈가 스윙 페이즈인지 스탠스 페이즈인지 구분 후 com_motion_time, foot_motion_time 계산 후 궤적 생성
    switch(walking.phase){
        case TROT_SWING:

            // RRFL 의 스윙 페이즈(일반 트롯)
            if (walking.step.count <= walking.step.RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);
            }

            // RLFR 의 스윙 페이즈(일반 트롯)
            else if (walking.step.count <= walking.step.FINAL_RLFR_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);
            }

            // RRFL 의 스윙 페이즈(트롯 종료버튼을 누른 후, 마지막 트롯)
            else if (walking.step.count <= walking.step.FINAL_RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
            }                      

            // RLFR 의 스윙 페이즈(트롯 종료버튼을 누른 후, 마지막 트롯)
            else {                      
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
                foot_motion_time                        = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
            }
            GainSchedule(walking.step.start_foot, foot_motion_time, walking.time.swing);

            generateCoMTraj(com_motion_time);    
            
            //void CRobot::generateFootTraj_Trot(float motion_time, int Startfoot) 
            generateFootTraj_Trot(foot_motion_time, walking.step.start_foot);

            //generateFootTraj_Stair(foot_motion_time, walking.step.start_foot);
            
            break;

        case TROT_STANCE:
            
            // RRFL 의 스탠스 페이즈(일반 트롯)
            if (walking.step.count <= walking.step.RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RLFR_SWING_START)*(float)(1.0/onesecScale);   
            }

            // RLFR 의 스탠스 페이즈(일반 트롯)
            else if (walking.step.count <= walking.step.FINAL_RLFR_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RRFL_SWING_START)*(float)(1.0/onesecScale);

//                if ((walking.step.count == walking.step.FINAL_RLFR_SWING_START -onesecScale*tasktime)&&(walking.Stop == false)){
                if ((walking.step.count == walking.step.FINAL_RLFR_SWING_START)&&(walking.Stop == false)){
//                    walking.step.count                  = walking.step.RLFR_SWING_START-onesecScale*tasktime;
                    walking.step.count                  = walking.step.RLFR_SWING_START;

                }
            }

            // RRFL 의 스윙 페이즈(트롯 종료버튼을 누른 후, 마지막 트롯)
            else if (walking.step.count <= walking.step.FINAL_RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)(1.0/onesecScale);
            }

            // RLFR 의 스윙 페이즈(트롯 종료버튼을 누른 후, 마지막 트롯)            
            else {            
                com_motion_time                         = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
            }

            
            
            GainSchedule(walking.step.start_foot, com_motion_time, walking.time.stance);   
            generateCoMTraj(com_motion_time);
            break;

        case TROT_NONE: 
            break;
    }
    
    if (walking.MoveDone == false){
        walking.step.count += onesecScale*tasktime;     // 현재 tasktime : 0.002[s], onesecScale = 1000 으로 정의되어 있기 때문에 카운트는 2씩 오른다.
    }
    

    //** 미래의 궤적들을 Trot 내부에서 만드는 게 낫겠다고 생각하게 되었다.
    //** 이유: 현재는 특정 버튼을 누르면 트롯이 동작하기 때문에, 다른 함수에서 트롯을 또 동작시키면 com_motion_time 등이 계속 커지는 골치가 생긴다.
    //** Trot 이 한번 호출될 때 미래 궤적까지 계산되도록 하는 게 바람직하다고 판단하였으며, 다리와 몸체의 미래 궤적들을 저장하기 위한 전역 변수를 새로 만들 필요가 생겼다.
    //VectorXf temp_com_ref = VectorXf::Zero(3); // 전역으로 만들기 전에 임시로

    //** ref 에 영벡터 대신 덮어씌우는 용도
    // for(size_t i = 0; i < 100; ++i){ // 100 은 미래를 바라보려는 갯수. 추후 Trot 함수의 디폴트 매개변수를 따로 만들어주어야 할 것이다.
    //     //temp_com_ref.block<3, 1>(i*3, 0) = generateCoMTraj_modified(com_motion_time, i+1);
    //     // temp_FR_ref.block<3, 1>(i*3, 0) = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, i+1);
    //     temp_legs_ref.block<3, 1>(i*3, 0) = FL_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 1) = FR_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 2) = RL_Foot.G_RefPos;
    //     temp_legs_ref.block<3, 1>(i*3, 3) = RR_Foot.G_RefPos;
    // }

    //** k = 0 일 때는 현재 시간에서의 값과 동일
    temp_com_ref.block<3, 1>(0, 0) = com.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 0) = FL_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 1) = FR_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 2) = RL_Foot.G_RefPos;
    temp_legs_ref.block<3, 1>(0, 3) = RR_Foot.G_RefPos;
    temp_com_vel_ref.block<3, 1>(0, 0) = com.G_RefVel;

    // std::cout << com.G_RefPos << std::endl;
    // std::cout << FL_Foot.G_RefPos << std::endl;
    temp_com2FL_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 0) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2FR_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 1) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2RL_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 2) - temp_com_ref.block<3, 1>(0, 0);
    temp_com2RR_ref.block<3, 1>(0, 0) = temp_legs_ref.block<3, 1>(0, 3) - temp_com_ref.block<3, 1>(0, 0);
    
    temp_contact_foots.block<4, 1>(0, 0) = contact.Foot;


    temp_com_vel_ref.block<3, 1>(0, 0) = com.G_RefVel;
    //** k > 0 일 때부터 미래 궤적을 계산하는 것이 필요
    for(size_t i = 1; i < 200; ++i){ // 100 은 미래를 바라보려는 갯수. 추후 Trot 함수의 디폴트 매개변수를 따로 만들어주어야 할 것이다.
        
        //temp_com_ref.block<3, 1>(i*3, 0) = generateCoMTraj_modified(com_motion_time, i+1);        
        //temp_legs_ref.block<3, 4>(i*3, 0) = generateFootTraj_Trot_modified(com_motion_time, foot_motion_time, walking.step.start_foot, walking.step.start_foot_prev, i+1);
        
        temp_com_ref.block<3, 1>(i*3, 0) = generateCoMTraj_modified(com_motion_time, i); //*(mpc_sampletime/tasktime)


        // 현재 여기서 죽는 중
        temp_legs_ref.block<3, 4>(i*3, 0) = generateFootTraj_Trot_modified(com_motion_time, foot_motion_time, walking.step.start_foot, walking.step.start_foot_prev, i);


        temp_com2FL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2FR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 1) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2RL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 2) - temp_com_ref.block<3, 1>(i*3, 0);
        temp_com2RR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 3) - temp_com_ref.block<3, 1>(i*3, 0);

        //temp_com_vel_ref.block<3, 1>((i-1)*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0))/(float)(2.0/onesecScale);
        
        temp_com_vel_ref.block<3, 1>(i*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0))/(float)(2.0/onesecScale);
        // temp_com_vel_ref.block<3, 1>(i*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0))/mpc_sampletime;
        
        
        //temp_com_vel_ref.block<3, 1>((i-1)*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0));
        //temp_com_vel_ref.block<3, 1>((i-1)*3, 0) << 1., 1., 1.;
        


        if(i == 199){
            temp_com_vel_ref.block<3, 1>(i*3, 0) = temp_com_vel_ref.block<3, 1>((i-1)*3, 0);
        }
        
    }
    // temp_com_vel_ref.block<3, 1>(0, 0) = com.G_RefVel;

    // for(size_t i = 1; i < 100; ++i){
    //     temp_com2FL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>(i*3, 0);
    //     temp_com2FR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 1) - temp_com_ref.block<3, 1>(i*3, 0);
    //     temp_com2RL_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 2) - temp_com_ref.block<3, 1>(i*3, 0);
    //     temp_com2RR_ref.block<3, 1>(i*3, 0) = temp_legs_ref.block<3, 1>(i*3, 3) - temp_com_ref.block<3, 1>(i*3, 0);

    //     //temp_com_vel_ref.block<3, 1>((i-1)*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0))*(float)(2.0/onesecScale);
    //     temp_com_vel_ref.block<3, 1>((i-1)*3, 0) = (temp_com_ref.block<3, 1>(i*3, 0) - temp_com_ref.block<3, 1>((i-1)*3, 0));
    //     //temp_com_vel_ref.block<3, 1>((i-1)*3, 0) << 1., 1., 1.;
    //     if(i == 99){
    //         temp_com_vel_ref.block<3, 1>(i*3, 0) = temp_com_vel_ref.block<3, 1>((i-1)*3, 0);
    //     }
    // }

    //VectorXf temp_com2FL_ref = VectorXf::Zero(3*100);
    //std::cout << "temp:com2FL_ref: \n" << temp_com2FL_ref << std::endl;



    // temp_FR_ref = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, 1);
    //temp_FR_ref2 = generateFootTraj_Trot_modified(foot_motion_time, walking.step.start_foot, 2);


    // plotjuggler 에서 찍어보기 위해, static 변수들을 잠시 전역 변수에 복사
    temp_com_motion_time = com_motion_time;
    temp_foot_motion_time = foot_motion_time;
    // std::cout << walking.step.count << std::endl;

    //void CRobot::OptimizationProcess(const VectorXf& com_target, WALK walk_para)
    //OptimizationProcess(walking.MovingSpeed, walking);
    //std::cout << "walking.time.swing: \n" << walking.time.swing << std::endl;

}

void CRobot::Trot_init(void) {
    /* Trot_init
     * Set Trot Walk Parameter for Trot Motion
     * Parameter : WalkingTime, PD Gain
     */
    walking.time.ready                                  = 0.10;
    walking.time.ready                                  = 0.;


    walking.time.stance                                 = 0.08;
    walking.time.swing                                  = 0.28;
    walking.time.step                                   = walking.time.stance                     +   walking.time.swing;
                              
    walking.step.FootHeight                             = 0.06; 
    walking.MoveDone                                    = false;
    walking.Stop                                        = false;
    
    // Swing(RLFR) -> Stance -> Swing(RRFL) -> Stance
    walking.step.RLFR_SWING_START                       = walking.time.ready*onesecScale;
    walking.step.RLFR_STANCE_START                      = walking.step.RLFR_SWING_START          + walking.time.swing*onesecScale;
    walking.step.RRFL_SWING_START                       = walking.step.RLFR_STANCE_START         + walking.time.stance*onesecScale;
    walking.step.RRFL_STANCE_START                      = walking.step.RRFL_SWING_START          + walking.time.swing*onesecScale;
             
    walking.step.FINAL_RLFR_SWING_START                 = walking.step.RRFL_STANCE_START         + walking.time.stance*onesecScale;
    walking.step.FINAL_RLFR_STANCE_START                = walking.step.FINAL_RLFR_SWING_START    + walking.time.swing*onesecScale;
    walking.step.FINAL_RRFL_SWING_START                 = walking.step.FINAL_RLFR_STANCE_START   + walking.time.stance*onesecScale;
    walking.step.FINAL_RRFL_STANCE_START                = walking.step.FINAL_RRFL_SWING_START    + walking.time.swing*onesecScale;
    walking.step.FINAL_RRFL_STANCE_END                  = walking.step.FINAL_RRFL_STANCE_START   + walking.time.stance*onesecScale;
                 
    walking.PreMovingSpeed                              <<    0,   0,   0;
             
    gain.InitJointKp                                    =   gain.JointKp;
    gain.InitJointKd                                    =   gain.JointKd;
    
    gain.InitBasePosKp                                  =   gain.BasePosKp;
    gain.InitBasePosKd                                  =   gain.BasePosKd;
    
    gain.InitTaskKp                                     =   gain.TaskKp;
    gain.InitTaskKd                                     =   gain.TaskKd;
}

void CRobot::Trot_stop(void) {
    setWalkingSpeedLimit();
}

void CRobot::StairWalk_Init(void) {
    /* Stair_Init
     * Set Trot Walk Parameter for Trot Motion
     * Parameter : WalkingTime, PD Gain
     */
    walking.time.ready                                  = 0.3;
    walking.time.stance                                 = 0.10;
    walking.time.swing                                  = 0.25;
    walking.time.step                                   = walking.time.stance                     +   walking.time.swing;
                         
    walking.step.FootHeight                             = 0.10; 
    walking.MoveDone                                    = false;
    walking.Stop                                        = false;

    // Swing(RLFR) -> Stance -> Swing(RRFL) -> Stance
    walking.step.RLFR_SWING_START                       = walking.time.ready*onesecScale;
    walking.step.RLFR_STANCE_START                      = walking.step.RLFR_SWING_START           + walking.time.swing*onesecScale;
    walking.step.RRFL_SWING_START                       = walking.step.RLFR_STANCE_START          + walking.time.stance*onesecScale;
    walking.step.RRFL_STANCE_START                      = walking.step.RRFL_SWING_START           + walking.time.swing*onesecScale;
             
    walking.step.FINAL_RLFR_SWING_START                 = walking.step.RRFL_STANCE_START          + walking.time.stance*onesecScale;
    walking.step.FINAL_RLFR_STANCE_START                = walking.step.FINAL_RLFR_SWING_START     + walking.time.swing*onesecScale;
    walking.step.FINAL_RRFL_SWING_START                 = walking.step.FINAL_RLFR_STANCE_START    + walking.time.stance*onesecScale;
    walking.step.FINAL_RRFL_STANCE_START                = walking.step.FINAL_RRFL_SWING_START     + walking.time.swing*onesecScale;
             
    walking.PreMovingSpeed                              <<    0,   0,   0;
             
    gain.InitJointKp                                    = gain.JointKp;
    gain.InitJointKd                                    = gain.JointKd;
    gain.InitTaskKp                                     = gain.TaskKp;
    gain.InitTaskKd                                     = gain.TaskKd;
}

void CRobot::StairWalk(void) {
    /* Stair Walking
     * Set CoM & Foot Position for Trot Motion
     */
    static float com_motion_time;
    static float foot_motion_time;

    if (walking.step.count == 0) {
        StairWalk_Init();
        walking.phase                                   = STAIR_READY;
    }
    
    if (walking.step.count == walking.step.RLFR_SWING_START) {
        walking.phase                                   = STAIR_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RLFR_STANCE_START) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = STAIR_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
    }
    else if (walking.step.count == walking.step.RRFL_SWING_START) {
        walking.phase                                   = STAIR_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
                             
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RRFL_STANCE_START) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = STAIR_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_SWING_START) {
        walking.phase                                   = STAIR_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
                     
        walking.MovingSpeed                             << 0, 0, 0;
                     
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_STANCE_START) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = STAIR_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_SWING_START) {
        walking.phase                                   = STAIR_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
                    
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_START) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = STAIR_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
        walking.MoveDone                                = true;
    }
    switch(walking.phase){

        case STAIR_SWING:

            if (walking.step.count < walking.step.RRFL_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RLFR_SWING_START)*(float)tasktime;
                foot_motion_time                        = (walking.step.count - walking.step.RLFR_SWING_START)*(float)tasktime;
            }
            else if (walking.step.count < walking.step.FINAL_RLFR_SWING_START){
                com_motion_time                         = (walking.step.count - walking.step.RRFL_SWING_START)*(float)tasktime;
                foot_motion_time                        = (walking.step.count - walking.step.RRFL_SWING_START)*(float)tasktime;
            }
            else if (walking.step.count < walking.step.FINAL_RRFL_SWING_START){
                com_motion_time      = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)tasktime;
                foot_motion_time     = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)tasktime;
            }
            else {
                com_motion_time     = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)tasktime;
                foot_motion_time    = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)tasktime;
            }

            GainSchedule(walking.step.start_foot, foot_motion_time, walking.time.swing);
            
            generateCoMTraj(com_motion_time);
            generateFootTraj_Stair(foot_motion_time, walking.step.start_foot);
            break;

        case STAIR_STANCE:

            if (walking.step.count < walking.step.RRFL_SWING_START){
                com_motion_time             = (walking.step.count - walking.step.RLFR_SWING_START)*(float)tasktime;
            }
            else if (walking.step.count < walking.step.FINAL_RLFR_SWING_START){
                com_motion_time             = (walking.step.count - walking.step.RRFL_SWING_START)*(float)tasktime;

                if ((walking.step.count == walking.step.FINAL_RLFR_SWING_START - 1)&&(walking.Stop == false)){
                    walking.step.count  = walking.step.RLFR_SWING_START - 1;

                }
            }
            else if (walking.step.count < walking.step.FINAL_RRFL_SWING_START){
                com_motion_time             = (walking.step.count - walking.step.FINAL_RLFR_SWING_START)*(float)tasktime;
            }
            else {
                com_motion_time             = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)tasktime;
            }

            GainSchedule(walking.step.start_foot, com_motion_time, walking.time.stance);
            
            generateCoMTraj(com_motion_time);
            break;

        case STAIR_NONE:

            break;
    }
    if (walking.MoveDone == false){
        walking.step.count++;
    }

}

void CRobot::GainSchedule(int StartFoot, float MotionTime, float PeriodTime) {
    /* GainSchedule
     * input : Mode, MotionTime, PeriodTime
     * output : Change PD Gain
     * Change the gain during Trot Motion
     */
    
    if (StartFoot       == walking.step.START_FOOT_RLFR){ 
        if(Mode == ACTUAL_ROBOT){
            gain.TargetJointKp  << 450,  450,  2800,
                                   900,  900,  5600,
                                   900,  900,  5600,
                                   450,  450,  2800;         

            gain.TargetJointKd  <<  50,   50,   300,
                                    50,   70,   350,
                                    50,   70,   350,
                                    50,   50,   300;
            
//            gain.TargetBasePosKp<<   40,   40,   40,
//                                     40,   40,   40;
//            
//            gain.TargetBasePosKd<<   4,    4,    4,
//                                     4,    4,    4;
            
        }
        else if(Mode == SIMULATION){            
            gain.TargetJointKp  << 450,  450,  2800,
                                   900,  900,  5600,
                                   900,  900,  5600,
                                   450,  450,  2800;         

            gain.TargetJointKd  <<  50,   50,   300,
                                    50,   70,   350,
                                    50,   70,   350,
                                    50,   50,   300;
            
//            gain.TargetBasePosKp<<   40,   40,   40,
//                                     40,   40,   40;
//            
//            gain.TargetBasePosKd<<   4,    4,    4,
//                                     4,    4,    4;
        }

        gain.TargetTaskKp       <<   0,     0,     0,
                                    80,   100,   100,
                                    80,   100,   100,
                                     0,     0,     0;

        gain.TargetTaskKd       <<   0,     0,     0,
                                     5,     5,     5,
                                     5,     5,     5,
                                     0,     0,     0;  

        
        gain.JointKp            = gain.TargetJointKp;
        gain.JointKd            = gain.TargetJointKd;

//        gain.BasePosKp          = gain.TargetBasePosKp;
//        gain.BasePosKd          = gain.TargetBasePosKd;
        
//        for(int nJoint = 0; nJoint < joint_DoF; ++nJoint){
//           gain.JointKp(nJoint)     = cosWave(gain.TargetJointKp(nJoint) -  gain.InitJointKp(nJoint),      PeriodTime/2,    MotionTime,   gain.InitJointKp(nJoint));
//           gain.JointKd(nJoint)     = cosWave(gain.TargetJointKd(nJoint) -  gain.InitJointKd(nJoint),      PeriodTime/2,    MotionTime,   gain.InitJointKd(nJoint));
//        }
    }
    else if(StartFoot       == walking.step.START_FOOT_RRFL){
        if(Mode == ACTUAL_ROBOT){            
            gain.TargetJointKp  <<  900,  900,  5600,
                                    450,  450,  2800,
                                    450,  450,  2800,       
                                    900,  900,  5600;         

            gain.TargetJointKd  <<  50,   70,   350,
                                    50,   50,   300,
                                    50,   50,   300,
                                    50,   70,   350;
            
//            gain.TargetBasePosKp<<   40,   40,   40,
//                                     40,   40,   40;
//            
//            gain.TargetBasePosKd<<   4,    4,    4,
//                                     4,    4,    4;
        }
        else if(Mode == SIMULATION){
            gain.TargetJointKp  <<  900,  900,  5600,
                                    450,  450,  2800,
                                    450,  450,  2800,       
                                    900,  900,  5600;         

            gain.TargetJointKd  <<  50,   70,   350,
                                    50,   50,   300,
                                    50,   50,   300,
                                    50,   70,   350;
            
//            gain.TargetBasePosKp<<   40,   40,   40,
//                                     40,   40,   40;
//            
//            gain.TargetBasePosKd<<   4,    4,    4,
//                                     4,    4,    4;
        }

        gain.TargetTaskKp   <<  80,   100,   100,
                                 0,     0,     0,
                                 0,     0,     0,
                                80,   100,   100;
 
        gain.TargetTaskKd   <<   5,     5,     5,
                                 0,     0,     0,
                                 0,     0,     0,
                                 5,     5,     5;  
    
        gain.JointKp            = gain.TargetJointKp;
        gain.JointKd            = gain.TargetJointKd;
         
//        gain.BasePosKp          = gain.TargetBasePosKp;
//        gain.BasePosKd          = gain.TargetBasePosKd;

//        for(int nJoint = 0; nJoint < joint_DoF; ++nJoint){
//            gain.JointKp(nJoint)     = cosWave(gain.TargetJointKp(nJoint) -  gain.InitJointKp(nJoint),      PeriodTime/2,    MotionTime,   gain.InitJointKp(nJoint));
//            gain.JointKd(nJoint)     = cosWave(gain.TargetJointKd(nJoint) -  gain.InitJointKd(nJoint),      PeriodTime/2,    MotionTime,   gain.InitJointKd(nJoint));
//        }
    }
    else if(StartFoot       == walking.step.NONE){
        gain.JointKp    =   gain.InitJointKp;
        gain.JointKd    =   gain.InitJointKd;
        gain.TaskKp     =   gain.InitJointKp;
        gain.TaskKd     =   gain.InitJointKd;
        
        gain.BasePosKp  =   gain.InitBasePosKp;
        gain.BasePosKd  =   gain.InitBasePosKd;
    } 
}

void CRobot::setWalkingSpeedLimit(){
    walking.MovingSpeed(AXIS_X)                         =   walking.trot.SpeedScale(AXIS_X)    *   walking.InputData[AXIS_X];
    walking.MovingSpeed(AXIS_Y)                         =   walking.trot.SpeedScale(AXIS_Y)    *   walking.InputData[AXIS_Y];
    walking.MovingSpeed(AXIS_YAW)                       =   walking.trot.SpeedScale(AXIS_YAW)  *   walking.InputData[AXIS_YAW];
    
    for (int AXIS = 0; AXIS < 3; AXIS++){
        if((walking.MovingSpeed(AXIS)-walking.PreMovingSpeed(AXIS))/walking.time.step > walking.trot.LimitAcceleration(AXIS)){
            walking.MovingSpeed(AXIS)                   = walking.PreMovingSpeed(AXIS) + walking.trot.LimitAcceleration(AXIS)*walking.time.step;
        }
        else if((walking.MovingSpeed(AXIS)-walking.PreMovingSpeed(AXIS))/walking.time.step < -walking.trot.LimitAcceleration(AXIS)){
            walking.MovingSpeed(AXIS)                   = walking.PreMovingSpeed(AXIS) - walking.trot.LimitAcceleration(AXIS)*walking.time.step;
        }
    }
    
    if((abs(walking.MovingSpeed(AXIS_X)) < walking.trot.ActiveThreshold)&&(abs(walking.MovingSpeed(AXIS_Y)) < walking.trot.ActiveThreshold)&&(abs(walking.MovingSpeed(AXIS_YAW)) < walking.trot.ActiveThreshold)){
        // walking.Stop = true;
    }       
    else if((abs(walking.MovingSpeed(AXIS_X)) > walking.trot.ActiveThreshold)||(abs(walking.MovingSpeed(AXIS_Y)) > walking.trot.ActiveThreshold)||(abs(walking.MovingSpeed(AXIS_YAW)) > walking.trot.ActiveThreshold))
    {
        if((walking.MoveDone         == true)&&(walking.step.count >= walking.step.FINAL_RRFL_STANCE_END)){
            walking.MoveDone                            = false;
            walking.Stop                                = false;
            walking.step.count                          = walking.step.RLFR_SWING_START-onesecScale*tasktime;
            
            if(CommandFlag == NONE_ACT){
                CommandFlag = TROT;
            }
        }
    }
}

// 기존 함수의 오버로딩
// Vector3f movesize_ref;
//setWalkingStep(movesize_ref);

//void CRobot::setWalkingStep(Vector3f& movesize)
void CRobot::setWalkingStep()
{
    float FBsize = walking.step.FBstepSize;
    float LRsize = walking.step.LRstepSize;
    float Turnsize = walking.step.TurnSize;
    int STartfoot = walking.step.start_foot;

    setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);

    //movesize = walking.step.com.MoveSize;
}
//setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);



void CRobot::setWalkingStep(float FBsize, float LRsize, float Turnsize, int Startfoot){
    /* setWalkingStep
     * input : PreFBsize, PreLRsize, FBsize, LRsize, Start foot, footHeight
     * output : Set Target CoM & Foot Position
     */

    //** FBsize = walking.step.FBstepSize;
    //** 내가 생각하는 함수의 입출력은 조금 다른데...
    //** 우선, 매개변수로 적혀있는 FBsize 등 모든 매개변수는 사실 walking 에 포함된 값이며
    //** 프로그램 내에서 이 함수는 100% setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
    //** 로 사용중이다.
    //** 따라서, setWalkingStep() 의 오버로딩 선언 후, 함수 내용에 FBsize, ... 등을 walking.step.FBstepSize 로 채운 뒤,
    //** CRobot::setWalkingStep(float FBsize, float LRsize, float Turnsize, int Startfoot) 를 호출하는 오버로딩 함수를 만들면 동일하게 작동한다..
    //** 따라서 별도의 입력 함수는 없다고 생각하더라도, CRobot 구조체의 어떤 성분들을 변경하는지 살펴보아야한다



    float** Base_TargetRotMat                       = matrix(1, 3, 1, 3);
    float** tempOutMat                              = matrix(1, 3, 1, 3);
    
    float** tempOutMat66                            = matrix(1, 6, 1, 6);
    float** OutMat66                                = matrix(1, 6, 1, 6);
    
    float* tempOutVec6                              = fvector(1, 6);
    float* OutVec6                                  = fvector(1, 6);
    
    float* MoveSize                                 = fvector(1, 3);
    float* PrePosition                              = fvector(1, 3);
    
    float* OutVec                                   = fvector(1, 3);
    float* tempOutVec                               = fvector(1, 3);
    
    if(Turnsize != 0)
    {
        if(Turnsize > 0)
        {
            walking.step.walking_Turn                = walking.step.TurnModeOn;
            walking.step.walking_TurnDirection       = walking.step.LeftTurn;
        }
        else if(Turnsize < 0)
        {
            walking.step.walking_Turn                = walking.step.TurnModeOn;
            walking.step.walking_TurnDirection       = walking.step.RightTurn;
        }
    }
    else
    {
        walking.step.walking_Turn                    = walking.step.TurnModeOff;
        walking.step.walking_TurnDirection           = walking.step.NoTurn;
    }
    
    walking.step.com.MoveSize(AXIS_X)                = FBsize/2;
    walking.step.com.MoveSize(AXIS_Y)                = LRsize/2;
    walking.step.base.Turnsize(AXIS_YAW)             = Turnsize/2;

    base.G_RefTargetOri(AXIS_ROLL)                   = base.G_RefOri(AXIS_ROLL);                 
    base.G_RefTargetOri(AXIS_PITCH)                  = slope.EstimatedAngle(AXIS_PITCH);            
    base.G_RefTargetOri(AXIS_YAW)                    = base.G_RefOri(AXIS_YAW) + walking.step.base.Turnsize(AXIS_YAW);
    
    setRotationZYX(base.G_RefTargetOri, Base_TargetRotMat);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            base.TargetRotMat(row, column)              = Base_TargetRotMat[row + 1][column + 1];
        }
    }

    base.G_RefTargetOri_dot(AXIS_ROLL)               = (base.G_RefTargetOri(AXIS_ROLL)  - base.G_RefOri(AXIS_ROLL))/walking.time.step; 
    base.G_RefTargetOri_dot(AXIS_PITCH)              = (base.G_RefTargetOri(AXIS_PITCH) - base.G_RefOri(AXIS_PITCH))/walking.time.step;
    base.G_RefTargetOri_dot(AXIS_YAW)                = (base.G_RefTargetOri(AXIS_YAW)   - base.G_RefOri(AXIS_YAW))/walking.time.step;

    //* Calculate E_ZYX
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        tempOutVec[AXIS + 1]                            = base.G_RefTargetOri_dot(AXIS);
    }
    setMappingE_ZYX(base.G_RefTargetOri, tempOutMat); 
    mvmult(tempOutMat, 3, 3, tempOutVec, 3, OutVec);  
        
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        base.G_RefTargetAngularVel(AXIS)                      = OutVec[AXIS + 1];
    }
    
    for (int i = 0; i < 3; ++i) {
        walking.PreState                                <<    base.G_RefOri(i),    base.G_RefAngularVel(i),    base.G_RefAngularAcc(i);
        walking.TargetState                             << base.G_RefTargetOri(i), base.G_RefTargetAngularVel(i), base.G_RefTargetAngularAcc(i);

        if (i == AXIS_ROLL){
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                base.Roll_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }
        }
        else if (i == AXIS_PITCH){
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                base.Pitch_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }
        }
        else if (i == AXIS_YAW){
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                base.Yaw_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }
        }
    }

    com.G_RefPrePos                                     = com.G_RefPos;
    com.G_RefPreVel                                     = com.G_RefVel;
    com.G_RefPreAcc                                     = com.G_RefAcc;
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        PrePosition[AXIS + 1]                           = com.G_RefPrePos(AXIS);
        MoveSize[AXIS + 1]                              = walking.step.com.MoveSize(AXIS);
    }
    mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
    vadd(PrePosition, 3, tempOutVec, OutVec);
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        com.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
    }
    
    com.G_RefTargetVel(AXIS_X)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_X);
    com.G_RefTargetVel(AXIS_Y)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_Y);
    com.G_RefTargetVel(AXIS_Z)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_Z);
    
//    com.G_RefTargetAcc(AXIS_X)               = ((com.G_RefTargetVel - com.G_RefPreVel)/walking.time.step)(AXIS_X);
//    com.G_RefTargetAcc(AXIS_Y)               = ((com.G_RefTargetVel - com.G_RefPreVel)/walking.time.step)(AXIS_Y);
//    com.G_RefTargetAcc(AXIS_Z)               = ((com.G_RefTargetVel - com.G_RefPreVel)/walking.time.step)(AXIS_Z);
    
    for (int i = 0; i < 3; ++i) {
        walking.PreState                <<    com.G_RefPrePos(i),    com.G_RefPreVel(i),    com.G_RefPreAcc(i);
        walking.TargetState             << com.G_RefTargetPos(i), com.G_RefTargetVel(i), com.G_RefTargetAcc(i);

        if (i == AXIS_X){
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                com.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }            
        }
        else if (i == AXIS_Y){
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                com.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }            
        }
        else if (i == AXIS_Z){        
            set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
            inverse(tempOutMat66, 6, OutMat66);
            for (int AXIS = 0; AXIS < 3; ++AXIS) {
                tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
            }
            mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
            for (int AXIS = 0; AXIS < 6; ++AXIS) {
                com.Z_Coeff[AXIS]                        = OutVec6[AXIS + 1];
            }            
        }
    }
   
    FL_Foot.G_RefPrePos                      = FL_Foot.G_RefPos;
    FL_Foot.G_RefPreVel                      = FL_Foot.G_RefVel;
    FL_Foot.G_RefPreAcc                      = FL_Foot.G_RefAcc;

    FR_Foot.G_RefPrePos                      = FR_Foot.G_RefPos;
    FR_Foot.G_RefPreVel                      = FR_Foot.G_RefVel;
    FR_Foot.G_RefPreAcc                      = FR_Foot.G_RefAcc;
    
    RL_Foot.G_RefPrePos                      = RL_Foot.G_RefPos;
    RL_Foot.G_RefPreVel                      = RL_Foot.G_RefVel;
    RL_Foot.G_RefPreAcc                      = RL_Foot.G_RefAcc;

    RR_Foot.G_RefPrePos                      = RR_Foot.G_RefPos;
    RR_Foot.G_RefPreVel                      = RR_Foot.G_RefVel;
    RR_Foot.G_RefPreAcc                      = RR_Foot.G_RefAcc;
            
    if (Startfoot       == walking.step.START_FOOT_RLFR){
        Vector3f CoM2Foot;
        contact.Foot[FL_FootIndex]      = true;
        contact.Foot[FR_FootIndex]      = false;
        contact.Foot[RL_FootIndex]      = false;
        contact.Foot[RR_FootIndex]      = true; 
    
        // RL Foot
        walking.step.RL_Foot.MoveSize(AXIS_X)   = (walking.MovingSpeed(AXIS_X) * walking.time.step)/2;
        walking.step.RL_Foot.MoveSize(AXIS_Y)   = (walking.MovingSpeed(AXIS_Y) * walking.time.step)/2; 
                 
        CoM2Foot                            << RL_Foot.G_InitPos(AXIS_X), RL_Foot.G_InitPos(AXIS_Y), -BodyKine.TargetCoMHeight;        
        
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            PrePosition[AXIS + 1]                           = com.G_RefTargetPos(AXIS);
            MoveSize[AXIS + 1]                              = (CoM2Foot + walking.step.RL_Foot.MoveSize)(AXIS);
        }
        mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
        vadd(PrePosition, 3, tempOutVec, OutVec);
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            RL_Foot.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
        }
      
        for (int i = 0; i < 3; ++i) {
            if (i == AXIS_X){
                walking.PreState                << RL_Foot.G_RefPrePos(i),    RL_Foot.G_RefPreVel(i),    RL_Foot.G_RefPreAcc(i);
                walking.TargetState             << RL_Foot.G_RefTargetPos(i), RL_Foot.G_RefTargetVel(i), RL_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RL_Foot.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Y){
                walking.PreState                << RL_Foot.G_RefPrePos(i),    RL_Foot.G_RefPreVel(i),    RL_Foot.G_RefPreAcc(i);
                walking.TargetState             << RL_Foot.G_RefTargetPos(i), RL_Foot.G_RefTargetVel(i), RL_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RL_Foot.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Z){     
                // UP
                walking.PreState_Up             <<                              RL_Foot.G_RefPrePos(i),      RL_Foot.G_RefPreVel(i),            RL_Foot.G_RefPreAcc(i);
                walking.TargetState_Up          << RL_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                          0,            -(float)GRAVITY;                
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Up(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Up(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RL_Foot.Z_Coeff_Up[AXIS]                        = OutVec6[AXIS + 1];
                }
                
                // DOWN                
                walking.PreState_Down           << RL_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                        0,               -(float)GRAVITY;
                walking.TargetState_Down        <<                           RL_Foot.G_RefTargetPos(i),     RL_Foot.G_RefTargetVel(i),          RL_Foot.G_RefTargetAcc(i);     
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Down(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Down(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RL_Foot.Z_Coeff_Down[AXIS]                        = OutVec6[AXIS + 1];
                }
            }   
        }
        
        // FR Foot
        walking.step.FR_Foot.MoveSize(AXIS_X)   = (walking.MovingSpeed(AXIS_X) * walking.time.step)/2;
        walking.step.FR_Foot.MoveSize(AXIS_Y)   = (walking.MovingSpeed(AXIS_Y) * walking.time.step)/2;  
        
        CoM2Foot                            << FR_Foot.G_InitPos(AXIS_X), FR_Foot.G_InitPos(AXIS_Y), -BodyKine.TargetCoMHeight;
        
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            PrePosition[AXIS + 1]                           = com.G_RefTargetPos(AXIS);
            MoveSize[AXIS + 1]                              = (CoM2Foot + walking.step.FR_Foot.MoveSize)(AXIS);
        }
        mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
        vadd(PrePosition, 3, tempOutVec, OutVec);
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            FR_Foot.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
        }
        
        for (int i = 0; i < 3; ++i) {
            if (i == AXIS_X){
                walking.PreState                << FR_Foot.G_RefPrePos(i),    FR_Foot.G_RefPreVel(i),    FR_Foot.G_RefPreAcc(i);
                walking.TargetState             << FR_Foot.G_RefTargetPos(i), FR_Foot.G_RefTargetVel(i), FR_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FR_Foot.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Y){
                walking.PreState                << FR_Foot.G_RefPrePos(i),    FR_Foot.G_RefPreVel(i),    FR_Foot.G_RefPreAcc(i);
                walking.TargetState             << FR_Foot.G_RefTargetPos(i), FR_Foot.G_RefTargetVel(i), FR_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FR_Foot.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Z){     
                // UP
                walking.PreState_Up             <<                              FR_Foot.G_RefPrePos(i),      FR_Foot.G_RefPreVel(i),            FR_Foot.G_RefPreAcc(i);
                walking.TargetState_Up          << FR_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                          0,            -(float)GRAVITY;                
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Up(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Up(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FR_Foot.Z_Coeff_Up[AXIS]                        = OutVec6[AXIS + 1];
                }
                
                // DOWN                
                walking.PreState_Down           << FR_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                        0,               -(float)GRAVITY;
                walking.TargetState_Down        <<                           FR_Foot.G_RefTargetPos(i),     FR_Foot.G_RefTargetVel(i),          FR_Foot.G_RefTargetAcc(i);                           
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Down(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Down(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FR_Foot.Z_Coeff_Down[AXIS]                        = OutVec6[AXIS + 1];
                }
            }   
        }
    }
    else if (Startfoot  == walking.step.START_FOOT_RRFL){
        Vector3f CoM2Foot;
        contact.Foot[FL_FootIndex]      = false;
        contact.Foot[FR_FootIndex]      = true;
        contact.Foot[RL_FootIndex]      = true;
        contact.Foot[RR_FootIndex]      = false;
  
        // FL Foot
        walking.step.FL_Foot.MoveSize(AXIS_X)   = (walking.MovingSpeed(AXIS_X) * walking.time.step)/2;
        walking.step.FL_Foot.MoveSize(AXIS_Y)   = (walking.MovingSpeed(AXIS_Y) * walking.time.step)/2;         
        
        CoM2Foot                            << FL_Foot.G_InitPos(AXIS_X), FL_Foot.G_InitPos(AXIS_Y), -BodyKine.TargetCoMHeight;    
        
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            PrePosition[AXIS + 1]                           = com.G_RefTargetPos(AXIS);
            MoveSize[AXIS + 1]                              = (CoM2Foot + walking.step.FL_Foot.MoveSize)(AXIS);
        }
        mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
        vadd(PrePosition, 3, tempOutVec, OutVec);
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            FL_Foot.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
        }
        
        for (int i = 0; i < 3; ++i) {
            if (i == AXIS_X){
                walking.PreState                << FL_Foot.G_RefPrePos(i),    FL_Foot.G_RefPreVel(i),    FL_Foot.G_RefPreAcc(i);
                walking.TargetState             << FL_Foot.G_RefTargetPos(i), FL_Foot.G_RefTargetVel(i), FL_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FL_Foot.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Y){
                walking.PreState                << FL_Foot.G_RefPrePos(i),    FL_Foot.G_RefPreVel(i),    FL_Foot.G_RefPreAcc(i);
                walking.TargetState             << FL_Foot.G_RefTargetPos(i), FL_Foot.G_RefTargetVel(i), FL_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FL_Foot.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Z){     
                // UP
                walking.PreState_Up             <<                              FL_Foot.G_RefPrePos(i),      FL_Foot.G_RefPreVel(i),            FL_Foot.G_RefPreAcc(i);
                walking.TargetState_Up          << FL_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                          0,            -(float)GRAVITY;
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Up(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Up(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FL_Foot.Z_Coeff_Up[AXIS]                        = OutVec6[AXIS + 1];
                }
                
                // DOWN                
                walking.PreState_Down           << FL_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                        0,               -(float)GRAVITY;
                walking.TargetState_Down        <<                           FL_Foot.G_RefTargetPos(i),     FL_Foot.G_RefTargetVel(i),          FL_Foot.G_RefTargetAcc(i);                
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Down(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Down(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    FL_Foot.Z_Coeff_Down[AXIS]                        = OutVec6[AXIS + 1];
                }
            }   
        }
        
        // RR Foot
        walking.step.RR_Foot.MoveSize(AXIS_X)   = (walking.MovingSpeed(AXIS_X) * walking.time.step)/2;
        walking.step.RR_Foot.MoveSize(AXIS_Y)   = (walking.MovingSpeed(AXIS_Y) * walking.time.step)/2; 
        
        CoM2Foot                            << RR_Foot.G_InitPos(AXIS_X), RR_Foot.G_InitPos(AXIS_Y), -BodyKine.TargetCoMHeight;    
        
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            PrePosition[AXIS + 1]                           = com.G_RefTargetPos(AXIS);
            MoveSize[AXIS + 1]                              = (CoM2Foot + walking.step.RR_Foot.MoveSize)(AXIS);
        }
        mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
        vadd(PrePosition, 3, tempOutVec, OutVec);
        for (int AXIS = 0; AXIS < 3; ++AXIS) {
            RR_Foot.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
        }
        
        for (int i = 0; i < 3; ++i) {
            if (i == AXIS_X){
                walking.PreState                << RR_Foot.G_RefPrePos(i),    RR_Foot.G_RefPreVel(i),    RR_Foot.G_RefPreAcc(i);
                walking.TargetState             << RR_Foot.G_RefTargetPos(i), RR_Foot.G_RefTargetVel(i), RR_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RR_Foot.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Y){
                walking.PreState                << RR_Foot.G_RefPrePos(i),    RR_Foot.G_RefPreVel(i),    RR_Foot.G_RefPreAcc(i);
                walking.TargetState             << RR_Foot.G_RefTargetPos(i), RR_Foot.G_RefTargetVel(i), RR_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RR_Foot.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
                }            
            }
            else if (i == AXIS_Z){     
                // UP
                walking.PreState_Up             <<                              RR_Foot.G_RefPrePos(i),      RR_Foot.G_RefPreVel(i),            RR_Foot.G_RefPreAcc(i);
                walking.TargetState_Up          << RR_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                          0,            -(float)GRAVITY;

                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Up(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Up(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RR_Foot.Z_Coeff_Up[AXIS]                        = OutVec6[AXIS + 1];
                }
                
                // DOWN                
                walking.PreState_Down           << RR_Foot.G_RefTargetPos(i) + walking.step.FootHeight,                        0,               -(float)GRAVITY;
                walking.TargetState_Down        <<                           RR_Foot.G_RefTargetPos(i),     RR_Foot.G_RefTargetVel(i),          RR_Foot.G_RefTargetAcc(i);
                
                set_5thPolyNomialMatrix(walking.time.swing/2.0, tempOutMat66);
                inverse(tempOutMat66, 6, OutMat66);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                    tempOutVec6[2*AXIS + 1]                       = walking.PreState_Down(AXIS);
                    tempOutVec6[2*AXIS + 2]                       = walking.TargetState_Down(AXIS);  
                }
                mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
                for (int AXIS = 0; AXIS < 6; ++AXIS) {
                    RR_Foot.Z_Coeff_Down[AXIS]                        = OutVec6[AXIS + 1];
                }
            }   
        }
    }
    
    free_matrix(Base_TargetRotMat, 1, 3, 1, 3);
    free_matrix(tempOutMat, 1, 3, 1, 3);
    free_vector(MoveSize, 1, 3);
    free_vector(PrePosition, 1, 3);
    free_vector(OutVec, 1, 3);
    free_vector(tempOutVec, 1, 3);    
    
    free_matrix(tempOutMat66, 1, 6, 1, 6);
    free_matrix(OutMat66, 1, 6, 1, 6);
    free_vector(tempOutVec6, 1, 6);
    free_vector(OutVec6, 1, 6);
    
}

void CRobot::generateCoMTraj(float motion_time) {
    /* generateCoMTraj
     * input : motion_time
     * output : Generate CoM Trajectory
     */    
    com.G_RefPos(AXIS_X)               = Function_5thPolyNomial(com.X_Coeff, motion_time);
    com.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(com.X_Coeff, motion_time);
    com.G_RefAcc(AXIS_X)               = Function_5thPolyNomial_2dot(com.X_Coeff, motion_time);

    com.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(com.Y_Coeff, motion_time);
    com.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(com.Y_Coeff, motion_time);
    com.G_RefAcc(AXIS_Y)               = Function_5thPolyNomial_2dot(com.Y_Coeff, motion_time);

    com.G_RefPos(AXIS_Z)               = Function_5thPolyNomial(com.Z_Coeff, motion_time);
    com.G_RefVel(AXIS_Z)               = Function_5thPolyNomial_dot(com.Z_Coeff, motion_time);
    com.G_RefAcc(AXIS_Z)               = Function_5thPolyNomial_2dot(com.Z_Coeff, motion_time);

    base.G_RefOri(AXIS_ROLL)           = Function_5thPolyNomial(base.Roll_Coeff, motion_time);
    base.G_RefOri(AXIS_PITCH)          = Function_5thPolyNomial(base.Pitch_Coeff, motion_time);
    base.G_RefOri(AXIS_YAW)            = Function_5thPolyNomial(base.Yaw_Coeff, motion_time);

    base.G_RefAngularVel(AXIS_ROLL)    = Function_5thPolyNomial_dot(base.Roll_Coeff, motion_time);
    base.G_RefAngularVel(AXIS_PITCH)   = Function_5thPolyNomial_dot(base.Pitch_Coeff, motion_time);
    base.G_RefAngularVel(AXIS_YAW)     = Function_5thPolyNomial_dot(base.Yaw_Coeff, motion_time);
    
    base.G_RefAngularAcc(AXIS_ROLL)    = Function_5thPolyNomial_2dot(base.Roll_Coeff, motion_time);
    base.G_RefAngularAcc(AXIS_PITCH)   = Function_5thPolyNomial_2dot(base.Pitch_Coeff, motion_time);
    base.G_RefAngularAcc(AXIS_YAW)     = Function_5thPolyNomial_2dot(base.Yaw_Coeff, motion_time);
}

void CRobot::generateFootTraj_Trot(float motion_time, int Startfoot) {
    /* generateFootTraj
     * input : motion_time, Start foot
     * output : Generate Foot Trajectory
     */
    
    if (Startfoot       == walking.step.START_FOOT_RLFR){
        FR_Foot.G_RefPos(AXIS_X)               = Function_5thPolyNomial(FR_Foot.X_Coeff, motion_time);
        FR_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(FR_Foot.X_Coeff, motion_time);

        FR_Foot.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(FR_Foot.Y_Coeff, motion_time);
        FR_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(FR_Foot.Y_Coeff, motion_time);
        
        RL_Foot.G_RefPos(AXIS_X)               = Function_5thPolyNomial(RL_Foot.X_Coeff, motion_time);
        RL_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(RL_Foot.X_Coeff, motion_time);

        RL_Foot.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(RL_Foot.Y_Coeff, motion_time);
        RL_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(RL_Foot.Y_Coeff, motion_time);


        if(motion_time < walking.time.swing/2.0){
            FR_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(FR_Foot.Z_Coeff_Up, motion_time);
            FR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FR_Foot.Z_Coeff_Up, motion_time);
            
            RL_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(RL_Foot.Z_Coeff_Up, motion_time);
            RL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RL_Foot.Z_Coeff_Up, motion_time);
        }
        else{
            FR_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(FR_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);
            FR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FR_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);            
            
            RL_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(RL_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);
            RL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RL_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);            
        }        
    }

    else if (Startfoot  == walking.step.START_FOOT_RRFL){
        
        FL_Foot.G_RefPos(AXIS_X)               = Function_5thPolyNomial(FL_Foot.X_Coeff, motion_time);
        FL_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(FL_Foot.X_Coeff, motion_time);

        FL_Foot.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(FL_Foot.Y_Coeff, motion_time);
        FL_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(FL_Foot.Y_Coeff, motion_time);
        
        RR_Foot.G_RefPos(AXIS_X)               = Function_5thPolyNomial(RR_Foot.X_Coeff, motion_time);
        RR_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(RR_Foot.X_Coeff, motion_time);

        RR_Foot.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(RR_Foot.Y_Coeff, motion_time);
        RR_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(RR_Foot.Y_Coeff, motion_time);
        
        
        if(motion_time < walking.time.swing/2.0){
            FL_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(FL_Foot.Z_Coeff_Up, motion_time);
            FL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FL_Foot.Z_Coeff_Up, motion_time);
            
            RR_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(RR_Foot.Z_Coeff_Up, motion_time);
            RR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RR_Foot.Z_Coeff_Up, motion_time);
        }
        else{            
            FL_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(FL_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);
            FL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FL_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);            
            
            RR_Foot.G_RefPos(AXIS_Z)           = Function_5thPolyNomial(RR_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);
            RR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RR_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);            
        }  
    }
}

void CRobot::generateFootTraj_Stair(float motion_time, int Startfoot) {
    /* generateFootTraj
     * input : motion_time, Start foot
     * output : Generate Foot Trajectory
     */    
    
    if (Startfoot       == walking.step.START_FOOT_RLFR){
        FR_Foot.ControlPointX               =      Set_8thBezierControlPoint(FR_Foot.G_RefPrePos, FR_Foot.G_RefTargetPos, AXIS_X, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        FR_Foot.ControlPointY               =      Set_8thBezierControlPoint(FR_Foot.G_RefPrePos, FR_Foot.G_RefTargetPos, AXIS_Y, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        FR_Foot.ControlPointZ               =      Set_8thBezierControlPoint(FR_Foot.G_RefPrePos, FR_Foot.G_RefTargetPos, AXIS_Z, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        
        FR_Foot.G_RefPos(AXIS_X)            =      Function_8thBezierCurve(                                FR_Foot.ControlPointX,       walking.time.swing,     motion_time);
        FR_Foot.G_RefPos(AXIS_Y)            =      Function_8thBezierCurve(                                FR_Foot.ControlPointY,       walking.time.swing,     motion_time);
        FR_Foot.G_RefPos(AXIS_Z)            =      Function_8thBezierCurve(                                FR_Foot.ControlPointZ,       walking.time.swing,     motion_time)
                                                + cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time,      0);
        FR_Foot.G_RefVel(AXIS_X)            =      Function_8thBezierCurve_dot(                                FR_Foot.ControlPointX,       walking.time.swing,     motion_time);
        FR_Foot.G_RefVel(AXIS_Y)            =         differential_cosWave(    FR_Foot.G_RefTargetPos(AXIS_Y) - FR_Foot.G_RefPrePos(AXIS_Y),       walking.time.swing,     motion_time);
        FR_Foot.G_RefVel(AXIS_Z)            =         differential_cosWave(    FR_Foot.G_RefTargetPos(AXIS_Z) - FR_Foot.G_RefPrePos(AXIS_Z),       walking.time.swing,     motion_time)
                                                     + differential_cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time);
        
        RL_Foot.ControlPointX               =      Set_8thBezierControlPoint(RL_Foot.G_RefPrePos, RL_Foot.G_RefTargetPos, AXIS_X, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        RL_Foot.ControlPointY               =      Set_8thBezierControlPoint(RL_Foot.G_RefPrePos, RL_Foot.G_RefTargetPos, AXIS_Y, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        RL_Foot.ControlPointZ               =      Set_8thBezierControlPoint(RL_Foot.G_RefPrePos, RL_Foot.G_RefTargetPos, AXIS_Z, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        
        RL_Foot.G_RefPos(AXIS_X)            =      Function_8thBezierCurve(                                RL_Foot.ControlPointX,       walking.time.swing,     motion_time);
        RL_Foot.G_RefPos(AXIS_Y)            =      Function_8thBezierCurve(                                RL_Foot.ControlPointY,       walking.time.swing,     motion_time);
        RL_Foot.G_RefPos(AXIS_Z)            =      Function_8thBezierCurve(                                RL_Foot.ControlPointZ,       walking.time.swing,     motion_time)
                                                + cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time,      0);
        
        RL_Foot.G_RefVel(AXIS_X)            =      Function_8thBezierCurve_dot(                                RL_Foot.ControlPointX,       walking.time.swing,     motion_time);
        RL_Foot.G_RefVel(AXIS_Y)            =         differential_cosWave(    RL_Foot.G_RefTargetPos(AXIS_Y) - RL_Foot.G_RefPrePos(AXIS_Y),       walking.time.swing,     motion_time);
        RL_Foot.G_RefVel(AXIS_Z)            =         differential_cosWave(    RL_Foot.G_RefTargetPos(AXIS_Z) - RL_Foot.G_RefPrePos(AXIS_Z),       walking.time.swing,     motion_time) 
                                                 + differential_cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time);

    }
    else if (Startfoot  == walking.step.START_FOOT_RRFL){
        FL_Foot.ControlPointX               =      Set_8thBezierControlPoint(FL_Foot.G_RefPrePos, FL_Foot.G_RefTargetPos, AXIS_X, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        FL_Foot.ControlPointY               =      Set_8thBezierControlPoint(FL_Foot.G_RefPrePos, FL_Foot.G_RefTargetPos, AXIS_Y, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        FL_Foot.ControlPointZ               =      Set_8thBezierControlPoint(FL_Foot.G_RefPrePos, FL_Foot.G_RefTargetPos, AXIS_Z, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        
        FL_Foot.G_RefPos(AXIS_X)            =      Function_8thBezierCurve(                                FL_Foot.ControlPointX,       walking.time.swing,     motion_time);
        FL_Foot.G_RefPos(AXIS_Y)            =      Function_8thBezierCurve(                                FL_Foot.ControlPointY,       walking.time.swing,     motion_time);
        FL_Foot.G_RefPos(AXIS_Z)            =      Function_8thBezierCurve(                                FL_Foot.ControlPointZ,       walking.time.swing,     motion_time)
                                                + cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time,      0);

        FL_Foot.G_RefVel(AXIS_X)            =      Function_8thBezierCurve_dot(                                FL_Foot.ControlPointX,       walking.time.swing,     motion_time);
        FL_Foot.G_RefVel(AXIS_Y)            =         differential_cosWave(    FL_Foot.G_RefTargetPos(AXIS_Y) - FL_Foot.G_RefPrePos(AXIS_Y),       walking.time.swing,     motion_time);
        FL_Foot.G_RefVel(AXIS_Z)            =         differential_cosWave(    FL_Foot.G_RefTargetPos(AXIS_Z) - FL_Foot.G_RefPrePos(AXIS_Z),       walking.time.swing,     motion_time)                                     
                                                + differential_cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time);

        
        RR_Foot.ControlPointX               =      Set_8thBezierControlPoint(RR_Foot.G_RefPrePos, RR_Foot.G_RefTargetPos, AXIS_X, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        RR_Foot.ControlPointY               =      Set_8thBezierControlPoint(RR_Foot.G_RefPrePos, RR_Foot.G_RefTargetPos, AXIS_Y, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));
        RR_Foot.ControlPointZ               =      Set_8thBezierControlPoint(RR_Foot.G_RefPrePos, RR_Foot.G_RefTargetPos, AXIS_Z, Z_Rot(base.G_RefTargetOri(AXIS_YAW),3));

        RR_Foot.G_RefPos(AXIS_X)            =      Function_8thBezierCurve(                                RR_Foot.ControlPointX,       walking.time.swing,     motion_time);
        RR_Foot.G_RefPos(AXIS_Y)            =      Function_8thBezierCurve(                                RR_Foot.ControlPointY,       walking.time.swing,     motion_time);
        RR_Foot.G_RefPos(AXIS_Z)            =      Function_8thBezierCurve(                                RR_Foot.ControlPointZ,       walking.time.swing,     motion_time)
                                                + cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time,      0);

        RR_Foot.G_RefVel(AXIS_X)            =      Function_8thBezierCurve_dot(                                RR_Foot.ControlPointX,       walking.time.swing,     motion_time);
        RR_Foot.G_RefVel(AXIS_Y)            =         differential_cosWave(    RR_Foot.G_RefTargetPos(AXIS_Y) - RR_Foot.G_RefPrePos(AXIS_Y),       walking.time.swing,     motion_time);
        RR_Foot.G_RefVel(AXIS_Z)            =         differential_cosWave(    RR_Foot.G_RefTargetPos(AXIS_Z) - RR_Foot.G_RefPrePos(AXIS_Z),       walking.time.swing,     motion_time)
                                             + differential_cosWave(                               walking.step.FootHeight,     walking.time.swing/2,     motion_time);
    }
}

void CRobot::SlopeEstimation(void) {
    /* SlopeEstimation
     * Estimate Slope Orientation
     */
    static float tmp_Estimated_Roll, pre_Estimated_Roll;
    static float tmp_Estimated_Pitch, pre_Estimated_Pitch;
    
    const double Offset_Coefficient             = 0.25;
        
    static float Estimated_Roll                 = 0.0;
    static float Estimated_Pitch                = 0.0;
    
    static int DegreeThreshold;
    const float UpdateWeight                    = 0.01; 

    if(Mode == ACTUAL_ROBOT){
        DegreeThreshold = 0.0;
    }
    else if(Mode == SIMULATION){
        DegreeThreshold = 0.0;
    }
    
    if ((contact.Foot(0) == 1)&&(contact.Foot(1) == 1)&&(contact.Foot(2) == 1)&&(contact.Foot(3) == 1)){
        if(ControlMode == CTRLMODE_TROT){
            if((imu.Ori(AXIS_PITCH) > -DegreeThreshold*D2R)&&(imu.Ori(AXIS_PITCH) < DegreeThreshold*D2R)){
                slope.EstimatedAngle(AXIS_PITCH)        = 0; 
            }
            else{
                float** OutMat                                  = matrix(1, 3, 1, 3);
                float** tempOutMat33                            = matrix(1, 3, 1, 3);
                float** tempOutMat34                            = matrix(1, 3, 1, 4);

                float* OutVec                                   = fvector(1, 3);
                float* tempOutVec                               = fvector(1, 4);           

                slope.FootState_X                       << FL_Foot.B_CurrentPos[0], FR_Foot.B_CurrentPos[0], RL_Foot.B_CurrentPos[0], RR_Foot.B_CurrentPos[0];
                slope.FootState_Y                       << FL_Foot.B_CurrentPos[1], FR_Foot.B_CurrentPos[1], RL_Foot.B_CurrentPos[1], RR_Foot.B_CurrentPos[1];
                slope.FootState_Z                       << FL_Foot.B_CurrentPos[2], FR_Foot.B_CurrentPos[2], RL_Foot.B_CurrentPos[2], RR_Foot.B_CurrentPos[2];
                
//                slope.FootState_X                       << FL_Foot.G_Base_to_Foot_CurrentPos[0], FR_Foot.G_Base_to_Foot_CurrentPos[0], RL_Foot.G_Base_to_Foot_CurrentPos[0], RR_Foot.G_Base_to_Foot_CurrentPos[0];
//                slope.FootState_Y                       << FL_Foot.G_Base_to_Foot_CurrentPos[1], FR_Foot.G_Base_to_Foot_CurrentPos[1], RL_Foot.G_Base_to_Foot_CurrentPos[1], RR_Foot.G_Base_to_Foot_CurrentPos[1];
//                slope.FootState_Z                       << FL_Foot.G_Base_to_Foot_CurrentPos[2], FR_Foot.G_Base_to_Foot_CurrentPos[2], RL_Foot.G_Base_to_Foot_CurrentPos[2], RR_Foot.G_Base_to_Foot_CurrentPos[2];
                
                slope.H.block(0, 0, 4, 1)               = - slope.FootState_X;
                slope.H.block(0, 1, 4, 1)               = - slope.FootState_Y;
                
                for (int row = 0; row < 4; ++row) {
                    for (int column = 0; column < 3; ++column) {
                        slope.H_Matrix[row + 1][column + 1]   = slope.H(row, column);
                    }
                }   
                mtranspose(slope.H_Matrix, 4, 3, slope.H_Matrix_transpose);
                
                for (int AXIS = 0; AXIS < 4; ++AXIS) {
                   tempOutVec[AXIS + 1]                         = slope.FootState_Z(AXIS);
                }
                
                mmult(slope.H_Matrix_transpose, 3, 4, slope.H_Matrix, 4, 3, tempOutMat33);
                inverse(tempOutMat33, 3, OutMat);
                mmult(OutMat, 3, 3, slope.H_Matrix_transpose, 3, 4, tempOutMat34);
                mvmult(tempOutMat34, 3, 4, tempOutVec, 4, OutVec);
                for (int AXIS = 0; AXIS < 3; ++AXIS) {
                   slope.LeastSquareSolution(AXIS)              = OutVec[AXIS + 1];
                }
                
                slope.normal << slope.LeastSquareSolution(0), slope.LeastSquareSolution(1), 1;

//                slope.V_vector = slope.H_Matrix.transpose() * slope.FootState_Z;
//                slope.TempLeastSquareSolution = (slope.H_Matrix.transpose() * slope.H_Matrix).inverse() * slope.V_vector;
                
                tmp_Estimated_Roll                      = -atan2(slope.LeastSquareSolution[1], 1) + imu.Ori(AXIS_ROLL);
                
                // Offset
                tmp_Estimated_Pitch                     = atan2(slope.LeastSquareSolution[0], 1) + imu.Ori(AXIS_PITCH);
                tmp_Estimated_Pitch                     = tmp_Estimated_Pitch - (Offset_Coefficient)*(tmp_Estimated_Pitch);
                

                Estimated_Roll                          = ((1 - UpdateWeight) * pre_Estimated_Roll  + UpdateWeight * tmp_Estimated_Roll);
                Estimated_Pitch                         = ((1 - UpdateWeight) * pre_Estimated_Pitch + UpdateWeight * tmp_Estimated_Pitch);  

                slope.EstimatedAngle(AXIS_ROLL)         = Estimated_Roll;
                slope.EstimatedAngle(AXIS_PITCH)        = Estimated_Pitch;

                pre_Estimated_Roll                      = Estimated_Roll;
                pre_Estimated_Pitch                     = Estimated_Pitch;

                // rt_printf("TEST = %f\n", Estimated_Pitch);                

                free_matrix(OutMat, 1, 3, 1, 3);
                free_matrix(tempOutMat33, 1, 3, 1, 3);
                free_matrix(tempOutMat34, 1, 3, 1, 4);
                free_vector(OutVec, 1, 3);
                free_vector(tempOutVec, 1, 4);
            }
        }
    }
}

Vector12f CRobot::computeIK(Vector12f EndPoint) {
    /* computeIK
     * input : EndPoint
     * output : Create Reference Joint Angle using Inverse Kinematics
     */
    const float L1              = sqrt(pow(lowerbody.HR_TO_HP_LENGTH(AXIS_Y), 2) + pow(lowerbody.HR_TO_HP_LENGTH(AXIS_Z), 2));
    const float L2              = sqrt(pow(lowerbody.HP_TO_KN_LENGTH(AXIS_X), 2) + pow(lowerbody.HP_TO_KN_LENGTH(AXIS_Z), 2));
    const float L3              = sqrt(pow(lowerbody.KN_TO_EP_LENGTH(AXIS_X), 2) + pow(lowerbody.KN_TO_EP_LENGTH(AXIS_Z), 2));
    static int  MATRTIX_OFFSET  = 3;
    
    static float tmp_x          = 0;
    static float tmp_y          = 0;
    static float tmp_z          = 0;

    VectorXf tmp_HR_TO_EP(3);
    VectorXf tmp_joint(12);

    //FL Foot
    tmp_HR_TO_EP                <<  EndPoint[FL_FootIndex*MATRTIX_OFFSET + AXIS_X] - lowerbody.FL_BASE_TO_HR(AXIS_X),
                                    EndPoint[FL_FootIndex*MATRTIX_OFFSET + AXIS_Y] - lowerbody.FL_BASE_TO_HR(AXIS_Y),
                                    EndPoint[FL_FootIndex*MATRTIX_OFFSET + AXIS_Z] - lowerbody.FL_BASE_TO_HR(AXIS_Z);

    tmp_x                       = tmp_HR_TO_EP(AXIS_X);
    tmp_y                       = tmp_HR_TO_EP(AXIS_Y);
    tmp_z                       = tmp_HR_TO_EP(AXIS_Z);

    tmp_joint[FLHR]             = - PI / 2 + asin(tmp_y / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2)))) + acos(L1 / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2))));
    tmp_joint[FLHP]             = acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)))) + atan(-tmp_x / abs(tmp_z));
    tmp_joint[FLKN]             = - PI + acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(tmp_x, 2) - pow(tmp_y, 2) - pow(tmp_z, 2)) / (2 * L2 * L3));

    //FR Foot
    tmp_HR_TO_EP                <<  EndPoint[FR_FootIndex*MATRTIX_OFFSET + AXIS_X] - lowerbody.FR_BASE_TO_HR(AXIS_X),
                                    EndPoint[FR_FootIndex*MATRTIX_OFFSET + AXIS_Y] - lowerbody.FR_BASE_TO_HR(AXIS_Y),
                                    EndPoint[FR_FootIndex*MATRTIX_OFFSET + AXIS_Z] - lowerbody.FR_BASE_TO_HR(AXIS_Z);

    tmp_x                       = tmp_HR_TO_EP(AXIS_X);
    tmp_y                       = tmp_HR_TO_EP(AXIS_Y);
    tmp_z                       = tmp_HR_TO_EP(AXIS_Z);

    tmp_joint[FRHR]             =   PI / 2 + asin(tmp_y / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2)))) - acos(L1 / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2))));
    tmp_joint[FRHP]             = acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)))) + atan(-tmp_x / abs(tmp_z));
    tmp_joint[FRKN]             = - PI + acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(tmp_x, 2) - pow(tmp_y, 2) - pow(tmp_z, 2)) / (2 * L2 * L3));

    //RL Foot
    tmp_HR_TO_EP                <<  EndPoint[RL_FootIndex*MATRTIX_OFFSET + AXIS_X] - lowerbody.RL_BASE_TO_HR(AXIS_X),
                                    EndPoint[RL_FootIndex*MATRTIX_OFFSET + AXIS_Y] - lowerbody.RL_BASE_TO_HR(AXIS_Y),
                                    EndPoint[RL_FootIndex*MATRTIX_OFFSET + AXIS_Z] - lowerbody.RL_BASE_TO_HR(AXIS_Z);

    tmp_x                       = tmp_HR_TO_EP(AXIS_X);
    tmp_y                       = tmp_HR_TO_EP(AXIS_Y);
    tmp_z                       = tmp_HR_TO_EP(AXIS_Z);

    tmp_joint[RLHR]             = - PI / 2 + asin(tmp_y / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2)))) + acos(L1 / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2))));
    tmp_joint[RLHP]             = acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)))) + atan(-tmp_x / abs(tmp_z));
    tmp_joint[RLKN]             = - PI + acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(tmp_x, 2) - pow(tmp_y, 2) - pow(tmp_z, 2)) / (2 * L2 * L3));

    //RR Foot
    tmp_HR_TO_EP                <<  EndPoint[RR_FootIndex*MATRTIX_OFFSET + AXIS_X] - lowerbody.RR_BASE_TO_HR(AXIS_X),
                                    EndPoint[RR_FootIndex*MATRTIX_OFFSET + AXIS_Y] - lowerbody.RR_BASE_TO_HR(AXIS_Y),
                                    EndPoint[RR_FootIndex*MATRTIX_OFFSET + AXIS_Z] - lowerbody.RR_BASE_TO_HR(AXIS_Z);

    tmp_x                       = tmp_HR_TO_EP(AXIS_X);
    tmp_y                       = tmp_HR_TO_EP(AXIS_Y);
    tmp_z                       = tmp_HR_TO_EP(AXIS_Z);

    tmp_joint[RRHR]             =   PI / 2 + asin(tmp_y / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2)))) - acos(L1 / (sqrt(pow(tmp_y, 2) + pow(tmp_z, 2))));
    tmp_joint[RRHP]             = acos((-pow(L1, 2) + pow(L2, 2) - pow(L3, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)) / (2 * L2 * sqrt(-pow(L1, 2) + pow(tmp_x, 2) + pow(tmp_y, 2) + pow(tmp_z, 2)))) + atan(-tmp_x / abs(tmp_z));
    tmp_joint[RRKN]             = - PI + acos((pow(L1, 2) + pow(L2, 2) + pow(L3, 2) - pow(tmp_x, 2) - pow(tmp_y, 2) - pow(tmp_z, 2)) / (2 * L2 * L3));
    
    return tmp_joint;
}

void CRobot::StateUpdate(void) {
    /* StateUpdate
     * Update Current CoM & Foot Position
     * Function : StateEstimation(), SaveData()
     */ 
    RobotStateUpdate();
    referenceUpdate();
    kinematicsUpdate();
    GenerateFriction();
    MomentumObserver(70);
    SaveData();
}

void CRobot::RobotStateUpdate(void) {
    
        
    VectorXf EP_RefPos                  = VectorXf::Zero(12);
    VectorXf Joint_RefPos               = VectorXf::Zero(12);
    
    static float samplingtime           = tasktime; //[s]
    
    //Joint Information Update
    EP_RefPos                           <<       FL_Foot.B_RefPos,           FR_Foot.B_RefPos,           RL_Foot.B_RefPos,           RR_Foot.B_RefPos;
    Joint_RefPos                        = computeIK(EP_RefPos);    
    
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
            joint[nJoint].RefPos            = Joint_RefPos[nJoint];
            joint[nJoint].RefVel            = (joint[nJoint].RefPos - joint[nJoint].RefPrePos)/samplingtime;
            joint[nJoint].RefAcc            = (joint[nJoint].RefVel - joint[nJoint].RefPreVel)/samplingtime;
            
            joint[nJoint].RefPrePos         = joint[nJoint].RefPos;
            joint[nJoint].RefPreVel         = joint[nJoint].RefVel;
            joint[nJoint].RefPreAcc         = joint[nJoint].RefAcc;
    }
   
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
        joint[nJoint].CurrentAcc        = (joint[nJoint].CurrentVel - joint[nJoint].CurrentPreVel)/samplingtime;
        
        joint[nJoint].CurrentPrePos     = joint[nJoint].CurrentPos;
        joint[nJoint].CurrentPreVel     = joint[nJoint].CurrentVel;
    }
    
   
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
        RobotState(6 + nJoint)          = joint[nJoint].CurrentPos;
        RobotStatedot(6 + nJoint)       = joint[nJoint].CurrentVel;
        RobotState2dot(6 + nJoint)      = joint[nJoint].CurrentAcc;
    }
    
    // EndPoint G_CurrentPosition in Base Frame
    for (int nJoint = 0; nJoint < joint_DoF; ++nJoint) {
        RobotState_local(6 + nJoint)          = joint[nJoint].CurrentPos;
    }
    
    RobotState(0)                       = 0;    //base.G_Foot_to_Base_CurrentPos(AXIS_X);
    RobotState(1)                       = 0;    //base.G_Foot_to_Base_CurrentPos(AXIS_Y);
    RobotState(2)                       = 0;    //base.G_Foot_to_Base_CurrentPos(AXIS_Z);
    RobotState(3)                       = base.G_CurrentOri(AXIS_ROLL);
    RobotState(4)                       = base.G_CurrentOri(AXIS_PITCH);
    RobotState(5)                       = base.G_CurrentOri(AXIS_YAW);
   
    RobotStatedot(0)                    = 0;    //base.G_Foot_to_Base_CurrentVel(AXIS_X);
    RobotStatedot(1)                    = 0;    //base.G_Foot_to_Base_CurrentVel(AXIS_Y);
    RobotStatedot(2)                    = 0;    //base.G_Foot_to_Base_CurrentVel(AXIS_Z);
    RobotStatedot(3)                    = base.B_CurrentAngularVel(AXIS_ROLL); 
    RobotStatedot(4)                    = base.B_CurrentAngularVel(AXIS_PITCH);
    RobotStatedot(5)                    = base.B_CurrentAngularVel(AXIS_YAW);
    
    
    //* Set Quaternion
    base.G_CurrentQuat                  = Math::Quaternion::fromZYXAngles(((base.G_CurrentOri.cast <double> ()).tail(3)).reverse());
    Math::Quaternion BaseQuat(base.G_CurrentQuat);
    m_pModel->SetQuaternion(base.ID, BaseQuat, RobotState);
    
    //* Contact Number
    contact.Number = 0;
    
    for (int i = 0; i < 4; ++i) {
        if (contact.Foot[i] == CONTACT_ON) {
            contact.Number++;
        }
    }
    
}

void CRobot::referenceUpdate(void) {    
    float* tempOutVec                      = fvector(1, 3);
    float* OutVec                          = fvector(1, 3);
    
    setRotationZYX(base.G_RefOri, Ref_C_gb);
    mtranspose(Ref_C_gb, 3, 3, Ref_C_bg);
    
    //* Reference Base Position in Global Frame   
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        tempOutVec[AXIS + 1]                            = com.G_RefPos(AXIS);
    }
    
    mvmult(Ref_C_gb, 3, 3, B_CoM2Base_RefPos, 3, G_Base2CoM_RefPos);  
    vadd(G_Base2CoM_RefPos, 3, tempOutVec, OutVec);
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        base.G_RefPos(AXIS)                             = OutVec[AXIS + 1];
    }
    
    //* Orientation
    base.G_RefQuat                                      = Math::Quaternion::fromZYXAngles(((base.G_RefOri.cast <double> ()).tail(3)).reverse());
            
    // base.RefAngularVelocity
    tempOutVec[AXIS_ROLL  + 1]                          = base.G_RefAngularVel(AXIS_ROLL);
    tempOutVec[AXIS_PITCH + 1]                          = base.G_RefAngularVel(AXIS_PITCH);
    tempOutVec[AXIS_YAW   + 1]                          = base.G_RefAngularVel(AXIS_YAW);
    
    mvmult(Ref_C_bg, 3, 3, tempOutVec, 3, OutVec);
        
    base.B_RefAngularVel(AXIS_ROLL)                     = OutVec[AXIS_ROLL   + 1];
    base.B_RefAngularVel(AXIS_PITCH)                    = OutVec[AXIS_PITCH  + 1];
    base.B_RefAngularVel(AXIS_YAW)                      = OutVec[AXIS_YAW    + 1];
    
    base.B_RefAngularAcc                                = (base.B_RefAngularVel - base.B_RefPreAngularVel)/tasktime;
    base.B_RefPreAngularVel                             = base.B_RefAngularVel;
    
    // EndPoint RefPosition in Base Frame    
    base.G_RefVel                                       = (base.G_RefPos - base.G_RefPrePos)/tasktime;
    base.G_RefAcc                                       = (base.G_RefVel - base.G_RefPreVel)/tasktime;
    
    base.G_RefPrePos                                    = base.G_RefPos;
    base.G_RefPreVel                                    = base.G_RefVel;
    
    //* Reference CoM to Foot Position in Global Frame       
    FL_Foot.G_CoM_to_Foot_RefPos                        = FL_Foot.G_RefPos - com.G_RefPos;
    FR_Foot.G_CoM_to_Foot_RefPos                        = FR_Foot.G_RefPos - com.G_RefPos;
    RL_Foot.G_CoM_to_Foot_RefPos                        = RL_Foot.G_RefPos - com.G_RefPos;
    RR_Foot.G_CoM_to_Foot_RefPos                        = RR_Foot.G_RefPos - com.G_RefPos;
            
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_CoM2Foot_RefPos[AXIS + 1]             = FL_Foot.G_CoM_to_Foot_RefPos(AXIS);
        FR_Foot.G_CoM2Foot_RefPos[AXIS + 1]             = FR_Foot.G_CoM_to_Foot_RefPos(AXIS);
        RL_Foot.G_CoM2Foot_RefPos[AXIS + 1]             = RL_Foot.G_CoM_to_Foot_RefPos(AXIS);
        RR_Foot.G_CoM2Foot_RefPos[AXIS + 1]             = RR_Foot.G_CoM_to_Foot_RefPos(AXIS);
    }
    
    //* Reference CoM to Foot Position in Base Frame
    mvmult(Ref_C_bg, 3, 3, FL_Foot.G_CoM2Foot_RefPos, 3, FL_Foot.B_CoM2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, FR_Foot.G_CoM2Foot_RefPos, 3, FR_Foot.B_CoM2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, RL_Foot.G_CoM2Foot_RefPos, 3, RL_Foot.B_CoM2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, RR_Foot.G_CoM2Foot_RefPos, 3, RR_Foot.B_CoM2Foot_RefPos);
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.B_CoM_to_Foot_RefPos(AXIS)              = FL_Foot.B_CoM2Foot_RefPos[AXIS + 1];
        FR_Foot.B_CoM_to_Foot_RefPos(AXIS)              = FR_Foot.B_CoM2Foot_RefPos[AXIS + 1];
        RL_Foot.B_CoM_to_Foot_RefPos(AXIS)              = RL_Foot.B_CoM2Foot_RefPos[AXIS + 1];
        RR_Foot.B_CoM_to_Foot_RefPos(AXIS)              = RR_Foot.B_CoM2Foot_RefPos[AXIS + 1];
    }
    
    //* Reference Base to Foot Position in Global Frame   
    FL_Foot.G_Base_to_Foot_RefPos                       = FL_Foot.G_RefPos - base.G_RefPos;
    FR_Foot.G_Base_to_Foot_RefPos                       = FR_Foot.G_RefPos - base.G_RefPos;
    RL_Foot.G_Base_to_Foot_RefPos                       = RL_Foot.G_RefPos - base.G_RefPos;
    RR_Foot.G_Base_to_Foot_RefPos                       = RR_Foot.G_RefPos - base.G_RefPos;
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_Base2Foot_RefPos[AXIS + 1]            = FL_Foot.G_Base_to_Foot_RefPos(AXIS);
        FR_Foot.G_Base2Foot_RefPos[AXIS + 1]            = FR_Foot.G_Base_to_Foot_RefPos(AXIS);
        RL_Foot.G_Base2Foot_RefPos[AXIS + 1]            = RL_Foot.G_Base_to_Foot_RefPos(AXIS);
        RR_Foot.G_Base2Foot_RefPos[AXIS + 1]            = RR_Foot.G_Base_to_Foot_RefPos(AXIS);
    }
    
    //* Reference Base to Foot Position in Base Frame
    mvmult(Ref_C_bg, 3, 3, FL_Foot.G_Base2Foot_RefPos, 3, FL_Foot.B_Base2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, FR_Foot.G_Base2Foot_RefPos, 3, FR_Foot.B_Base2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, RL_Foot.G_Base2Foot_RefPos, 3, RL_Foot.B_Base2Foot_RefPos);
    mvmult(Ref_C_bg, 3, 3, RR_Foot.G_Base2Foot_RefPos, 3, RR_Foot.B_Base2Foot_RefPos);

    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.B_RefPos(AXIS)                          = FL_Foot.B_Base2Foot_RefPos[AXIS + 1];
        FR_Foot.B_RefPos(AXIS)                          = FR_Foot.B_Base2Foot_RefPos[AXIS + 1];
        RL_Foot.B_RefPos(AXIS)                          = RL_Foot.B_Base2Foot_RefPos[AXIS + 1];
        RR_Foot.B_RefPos(AXIS)                          = RR_Foot.B_Base2Foot_RefPos[AXIS + 1];
    }
    
    //* Reference CoM to Foot Velocity in Global Frame 
    static Vector3f FL_Foot_G_CoM_to_Foot_RefPrePos;
    static Vector3f FR_Foot_G_CoM_to_Foot_RefPrePos;
    static Vector3f RL_Foot_G_CoM_to_Foot_RefPrePos;
    static Vector3f RR_Foot_G_CoM_to_Foot_RefPrePos;
    
    FL_Foot.G_CoM_to_Foot_RefVel                        = (FL_Foot.G_CoM_to_Foot_RefPos - FL_Foot_G_CoM_to_Foot_RefPrePos)/tasktime;
    FR_Foot.G_CoM_to_Foot_RefVel                        = (FR_Foot.G_CoM_to_Foot_RefPos - FR_Foot_G_CoM_to_Foot_RefPrePos)/tasktime;
    RL_Foot.G_CoM_to_Foot_RefVel                        = (RL_Foot.G_CoM_to_Foot_RefPos - RL_Foot_G_CoM_to_Foot_RefPrePos)/tasktime;
    RR_Foot.G_CoM_to_Foot_RefVel                        = (RR_Foot.G_CoM_to_Foot_RefPos - RR_Foot_G_CoM_to_Foot_RefPrePos)/tasktime;

    
    FL_Foot_G_CoM_to_Foot_RefPrePos                     = FL_Foot.G_CoM_to_Foot_RefPos;
    FR_Foot_G_CoM_to_Foot_RefPrePos                     = FR_Foot.G_CoM_to_Foot_RefPos;
    RL_Foot_G_CoM_to_Foot_RefPrePos                     = RL_Foot.G_CoM_to_Foot_RefPos;
    RR_Foot_G_CoM_to_Foot_RefPrePos                     = RR_Foot.G_CoM_to_Foot_RefPos;
    
    //* Reference Base to Foot Velocity in Global Frame     
    static Vector3f FL_Foot_G_Base_to_Foot_RefPrePos;
    static Vector3f FR_Foot_G_Base_to_Foot_RefPrePos;
    static Vector3f RL_Foot_G_Base_to_Foot_RefPrePos;
    static Vector3f RR_Foot_G_Base_to_Foot_RefPrePos;

    FL_Foot.G_Base_to_Foot_RefVel                       = (FL_Foot.G_Base_to_Foot_RefPos - FL_Foot_G_Base_to_Foot_RefPrePos)/tasktime;
    FR_Foot.G_Base_to_Foot_RefVel                       = (FR_Foot.G_Base_to_Foot_RefPos - FR_Foot_G_Base_to_Foot_RefPrePos)/tasktime;
    RL_Foot.G_Base_to_Foot_RefVel                       = (RL_Foot.G_Base_to_Foot_RefPos - RL_Foot_G_Base_to_Foot_RefPrePos)/tasktime;
    RR_Foot.G_Base_to_Foot_RefVel                       = (RR_Foot.G_Base_to_Foot_RefPos - RR_Foot_G_Base_to_Foot_RefPrePos)/tasktime;
    
    FL_Foot_G_Base_to_Foot_RefPrePos                    = FL_Foot.G_Base_to_Foot_RefPos;
    FR_Foot_G_Base_to_Foot_RefPrePos                    = FR_Foot.G_Base_to_Foot_RefPos;
    RL_Foot_G_Base_to_Foot_RefPrePos                    = RL_Foot.G_Base_to_Foot_RefPos;
    RR_Foot_G_Base_to_Foot_RefPrePos                    = RR_Foot.G_Base_to_Foot_RefPos;
    
    //* Reference CoM to Foot Velocity in Base Frame
    static Vector3f FL_Foot_B_CoM_to_Foot_RefPrePos;
    static Vector3f FR_Foot_B_CoM_to_Foot_RefPrePos;
    static Vector3f RL_Foot_B_CoM_to_Foot_RefPrePos;
    static Vector3f RR_Foot_B_CoM_to_Foot_RefPrePos;
    
    FL_Foot.B_CoM_to_Foot_RefVel                        = (FL_Foot.B_CoM_to_Foot_RefPos - FL_Foot_B_CoM_to_Foot_RefPrePos)/tasktime;
    FR_Foot.B_CoM_to_Foot_RefVel                        = (FR_Foot.B_CoM_to_Foot_RefPos - FR_Foot_B_CoM_to_Foot_RefPrePos)/tasktime;
    RL_Foot.B_CoM_to_Foot_RefVel                        = (RL_Foot.B_CoM_to_Foot_RefPos - RL_Foot_B_CoM_to_Foot_RefPrePos)/tasktime;
    RR_Foot.B_CoM_to_Foot_RefVel                        = (RR_Foot.B_CoM_to_Foot_RefPos - RR_Foot_B_CoM_to_Foot_RefPrePos)/tasktime;    

    
    FL_Foot_B_CoM_to_Foot_RefPrePos                     = FL_Foot.B_CoM_to_Foot_RefPos;
    FR_Foot_B_CoM_to_Foot_RefPrePos                     = FR_Foot.B_CoM_to_Foot_RefPos;
    RL_Foot_B_CoM_to_Foot_RefPrePos                     = RL_Foot.B_CoM_to_Foot_RefPos;
    RR_Foot_B_CoM_to_Foot_RefPrePos                     = RR_Foot.B_CoM_to_Foot_RefPos;
    
    //* Reference Base to Foot Velocity in Base Frame
    static Vector3f FL_Foot_B_RefPrePos;
    static Vector3f FR_Foot_B_RefPrePos;
    static Vector3f RL_Foot_B_RefPrePos;
    static Vector3f RR_Foot_B_RefPrePos;
    
    FL_Foot.B_RefVel                                    = (FL_Foot.B_RefPos - FL_Foot_B_RefPrePos)/tasktime;
    FR_Foot.B_RefVel                                    = (FR_Foot.B_RefPos - FR_Foot_B_RefPrePos)/tasktime;
    RL_Foot.B_RefVel                                    = (RL_Foot.B_RefPos - RL_Foot_B_RefPrePos)/tasktime;
    RR_Foot.B_RefVel                                    = (RR_Foot.B_RefPos - RR_Foot_B_RefPrePos)/tasktime;

    base.G_Foot_to_Base_RefPos                          = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_Base_to_Foot_RefPos)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_Base_to_Foot_RefPos)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_Base_to_Foot_RefPos)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_Base_to_Foot_RefPos))/ contact.Number;
    base.G_Foot_to_Base_RefVel                          = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_Base_to_Foot_RefVel)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_Base_to_Foot_RefVel)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_Base_to_Foot_RefVel)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_Base_to_Foot_RefVel))/ contact.Number;
    
    com.G_Foot_to_CoM_RefPos                            = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_CoM_to_Foot_RefPos)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_CoM_to_Foot_RefPos)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_CoM_to_Foot_RefPos)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_CoM_to_Foot_RefPos))/ contact.Number;
    com.G_Foot_to_CoM_RefVel                            = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_CoM_to_Foot_RefVel)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_CoM_to_Foot_RefVel)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_CoM_to_Foot_RefVel)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_CoM_to_Foot_RefVel))/ contact.Number;
    
    FL_Foot_B_RefPrePos                                 = FL_Foot.B_RefPos;
    FR_Foot_B_RefPrePos                                 = FR_Foot.B_RefPos;
    RL_Foot_B_RefPrePos                                 = RL_Foot.B_RefPos;
    RR_Foot_B_RefPrePos                                 = RR_Foot.B_RefPos;
    
    free_vector(tempOutVec, 1, 3);
    free_vector(OutVec, 1, 3);

    //** CoM 의 레퍼런스 궤적에서, 글로벌 좌표계와 다리 좌표계 사이의 오프셋을 여기에서 계산해주도록 하자
    temp_Foot_to_CoM_vel_ref.block<3, 1>(0, 0) = com.G_Foot_to_CoM_RefVel;
    temp_Offset_Global_between_FootCoord = com.G_RefPos - com.G_Foot_to_CoM_RefPos;
    for(size_t i = 0; i < 100; ++i){
        temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0) = temp_com_ref.block<3, 1>(i*3, 0) - temp_Offset_Global_between_FootCoord;
        if (i > 1){
            temp_Foot_to_CoM_vel_ref.block<3, 1>(i*3, 0) = (temp_Foot_to_CoM_ref.block<3, 1>(i*3, 0) - temp_Foot_to_CoM_ref.block<3, 1>((i-1)*3, 0))/tasktime;
        }
    }
}

void CRobot::kinematicsUpdate(void) {
    //* For calculation
    float** OutMat                                      = matrix(1, 3, 1, 3);
    float** tempOutMat                                  = matrix(1, 3, 1, 3);
    float* OutVec                                       = fvector(1, 3);
    float* tempOutVec                                   = fvector(1, 3);

    IMU_Update();
    
    setRotationZYX(imu.Ori, Current_C_gb);
    mtranspose(Current_C_gb, 3, 3, Current_C_bg);
            
    // Calculate SkewSymmetric Matrix
    set_skew_symmetic(imu.AngularVel, Current_B_w_gb_skew);
    mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, OutMat);
    mmult(OutMat, 3, 3, Current_C_bg, 3, 3, Current_G_w_gb_skew);
    
    base.G_CurrentOri                                   <<        imu.Ori(AXIS_ROLL),        imu.Ori(AXIS_PITCH),        imu.Ori(AXIS_YAW);
    base.B_CurrentAngularVel                            << imu.AngularVel(AXIS_ROLL), imu.AngularVel(AXIS_PITCH),     imu.AngularVel(AXIS_YAW);
    
    tempOutVec[AXIS_ROLL  + 1]                          = base.B_CurrentAngularVel(AXIS_ROLL);
    tempOutVec[AXIS_PITCH + 1]                          = base.B_CurrentAngularVel(AXIS_PITCH);
    tempOutVec[AXIS_YAW   + 1]                          = base.B_CurrentAngularVel(AXIS_YAW);
    
    mvmult(Current_C_gb, 3, 3, tempOutVec, 3, OutVec);
    
    base.G_CurrentAngularVel(AXIS_ROLL)                 = OutVec[AXIS_ROLL   + 1];
    base.G_CurrentAngularVel(AXIS_PITCH)                = OutVec[AXIS_PITCH  + 1];
    base.G_CurrentAngularVel(AXIS_YAW)                  = OutVec[AXIS_YAW    + 1];

    setMappingE_ZYX(imu.Ori, Current_mapE_B);
    invMappingE_ZYX(imu.Ori, Current_mapE_B_inv);
    
    mmult(Current_C_gb, 3, 3, BodyKine.B_I_tensor, 3, 3, OutMat);
    mmult(OutMat, 3, 3, Current_C_bg, 3, 3, BodyKine.G_I_tensor);
    
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            BodyKine.G_I_g(row, column)   = BodyKine.G_I_tensor[row + 1][column + 1];
        }
    }
    
    // Calculate RobotState
    // Compute Analytical Jacobian
    // CalcPointJacobian6D(*m_pModel, RobotState_local, base.ID, Vector3d::Zero(), J_BASE, true);
    CalcPointJacobian(*m_pModel, RobotState_local, FL_Foot.ID, Vector3d::Zero(), J_FL, true);
    CalcPointJacobian(*m_pModel, RobotState_local, FR_Foot.ID, Vector3d::Zero(), J_FR, true);
    CalcPointJacobian(*m_pModel, RobotState_local, RL_Foot.ID, Vector3d::Zero(), J_RL, true);
    CalcPointJacobian(*m_pModel, RobotState_local, RR_Foot.ID, Vector3d::Zero(), J_RR, true);
    
    
    B_J_FL                                              << (J_FL.cast <float> ()).block<3, 3>(0, 6);
    B_J_FR                                              << (J_FR.cast <float> ()).block<3, 3>(0, 9);
    B_J_RL                                              << (J_RL.cast <float> ()).block<3, 3>(0, 12);
    B_J_RR                                              << (J_RR.cast <float> ()).block<3, 3>(0, 15);

    
    //* Current Base to CoM Position in Base Frame
//    com.G_Base_to_CoM_CurrentPos                                   = GetCOM(base.G_CurrentPos, base.G_CurrentOri, Joint_CurrentPos);         
    
    
    //* Current Base to Foot Position in Base Frame
    FL_Foot.B_CurrentPos                                = (CalcBodyToBaseCoordinates(*m_pModel, RobotState_local, FL_Foot.ID, Vector3d::Zero(), true)).cast <float> ();
    FR_Foot.B_CurrentPos                                = (CalcBodyToBaseCoordinates(*m_pModel, RobotState_local, FR_Foot.ID, Vector3d::Zero(), true)).cast <float> ();
    RL_Foot.B_CurrentPos                                = (CalcBodyToBaseCoordinates(*m_pModel, RobotState_local, RL_Foot.ID, Vector3d::Zero(), true)).cast <float> ();
    RR_Foot.B_CurrentPos                                = (CalcBodyToBaseCoordinates(*m_pModel, RobotState_local, RR_Foot.ID, Vector3d::Zero(), true)).cast <float> ();
    
    //* Current CoM to Foot Position in Base Frame
    FL_Foot.B_CoM_to_Foot_CurrentPos                    = FL_Foot.B_CurrentPos - com.B_Base_to_CoM_CurrentPos;
    FR_Foot.B_CoM_to_Foot_CurrentPos                    = FR_Foot.B_CurrentPos - com.B_Base_to_CoM_CurrentPos;
    RL_Foot.B_CoM_to_Foot_CurrentPos                    = RL_Foot.B_CurrentPos - com.B_Base_to_CoM_CurrentPos;
    RR_Foot.B_CoM_to_Foot_CurrentPos                    = RR_Foot.B_CurrentPos - com.B_Base_to_CoM_CurrentPos;
    
    //* Current Base to Foot Position in Global Frame
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.B_Base2Foot_CurrentPos[AXIS + 1]        = FL_Foot.B_CurrentPos(AXIS);
        FR_Foot.B_Base2Foot_CurrentPos[AXIS + 1]        = FR_Foot.B_CurrentPos(AXIS);
        RL_Foot.B_Base2Foot_CurrentPos[AXIS + 1]        = RL_Foot.B_CurrentPos(AXIS);
        RR_Foot.B_Base2Foot_CurrentPos[AXIS + 1]        = RR_Foot.B_CurrentPos(AXIS);
    }

    mvmult(Current_C_gb, 3, 3, FL_Foot.B_Base2Foot_CurrentPos, 3, FL_Foot.G_Base2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, FR_Foot.B_Base2Foot_CurrentPos, 3, FR_Foot.G_Base2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, RL_Foot.B_Base2Foot_CurrentPos, 3, RL_Foot.G_Base2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, RR_Foot.B_Base2Foot_CurrentPos, 3, RR_Foot.G_Base2Foot_CurrentPos);
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_Base_to_Foot_CurrentPos(AXIS)         = FL_Foot.G_Base2Foot_CurrentPos[AXIS + 1];
        FR_Foot.G_Base_to_Foot_CurrentPos(AXIS)         = FR_Foot.G_Base2Foot_CurrentPos[AXIS + 1];
        RL_Foot.G_Base_to_Foot_CurrentPos(AXIS)         = RL_Foot.G_Base2Foot_CurrentPos[AXIS + 1];
        RR_Foot.G_Base_to_Foot_CurrentPos(AXIS)         = RR_Foot.G_Base2Foot_CurrentPos[AXIS + 1];
    }
    
    //* Current CoM to Foot Position in Global Frame            
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.B_CoM2Foot_CurrentPos[AXIS + 1]         = FL_Foot.B_CoM_to_Foot_CurrentPos(AXIS);
        FR_Foot.B_CoM2Foot_CurrentPos[AXIS + 1]         = FR_Foot.B_CoM_to_Foot_CurrentPos(AXIS);
        RL_Foot.B_CoM2Foot_CurrentPos[AXIS + 1]         = RL_Foot.B_CoM_to_Foot_CurrentPos(AXIS);
        RR_Foot.B_CoM2Foot_CurrentPos[AXIS + 1]         = RR_Foot.B_CoM_to_Foot_CurrentPos(AXIS);
    }

    mvmult(Current_C_gb, 3, 3, FL_Foot.B_CoM2Foot_CurrentPos, 3, FL_Foot.G_CoM2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, FR_Foot.B_CoM2Foot_CurrentPos, 3, FR_Foot.G_CoM2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, RL_Foot.B_CoM2Foot_CurrentPos, 3, RL_Foot.G_CoM2Foot_CurrentPos);
    mvmult(Current_C_gb, 3, 3, RR_Foot.B_CoM2Foot_CurrentPos, 3, RR_Foot.G_CoM2Foot_CurrentPos);

    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_CoM_to_Foot_CurrentPos(AXIS)          = FL_Foot.G_CoM2Foot_CurrentPos[AXIS + 1];
        FR_Foot.G_CoM_to_Foot_CurrentPos(AXIS)          = FR_Foot.G_CoM2Foot_CurrentPos[AXIS + 1];
        RL_Foot.G_CoM_to_Foot_CurrentPos(AXIS)          = RL_Foot.G_CoM2Foot_CurrentPos[AXIS + 1];
        RR_Foot.G_CoM_to_Foot_CurrentPos(AXIS)          = RR_Foot.G_CoM2Foot_CurrentPos[AXIS + 1];
    }
    

    
    //* Current Base to Foot Velocity in Base Frame 
    static Vector3f FL_Foot_CurrentJointVel             = Vector3f::Zero();
    static Vector3f FR_Foot_CurrentJointVel             = Vector3f::Zero();
    static Vector3f RL_Foot_CurrentJointVel             = Vector3f::Zero();
    static Vector3f RR_Foot_CurrentJointVel             = Vector3f::Zero();
    
    FL_Foot_CurrentJointVel                             << joint[FLHR].CurrentVel, joint[FLHP].CurrentVel, joint[FLKN].CurrentVel;
    FR_Foot_CurrentJointVel                             << joint[FRHR].CurrentVel, joint[FRHP].CurrentVel, joint[FRKN].CurrentVel;
    RL_Foot_CurrentJointVel                             << joint[RLHR].CurrentVel, joint[RLHP].CurrentVel, joint[RLKN].CurrentVel;
    RR_Foot_CurrentJointVel                             << joint[RRHR].CurrentVel, joint[RRHP].CurrentVel, joint[RRKN].CurrentVel;
    
    FL_Foot.B_CurrentVel                                = B_J_FL*FL_Foot_CurrentJointVel;
    FR_Foot.B_CurrentVel                                = B_J_FR*FR_Foot_CurrentJointVel;
    RL_Foot.B_CurrentVel                                = B_J_RL*RL_Foot_CurrentJointVel;
    RR_Foot.B_CurrentVel                                = B_J_RR*RR_Foot_CurrentJointVel;
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.B_Base2Foot_CurrentVel[AXIS + 1]        = FL_Foot.B_CurrentVel(AXIS);
        FR_Foot.B_Base2Foot_CurrentVel[AXIS + 1]        = FR_Foot.B_CurrentVel(AXIS);
        RL_Foot.B_Base2Foot_CurrentVel[AXIS + 1]        = RL_Foot.B_CurrentVel(AXIS);
        RR_Foot.B_Base2Foot_CurrentVel[AXIS + 1]        = RR_Foot.B_CurrentVel(AXIS);
    }
    
    //* Current CoM to Foot Velocity in Base Frame 
    FL_Foot.B_CoM_to_Foot_CurrentVel                    = FL_Foot.B_CurrentVel;
    FR_Foot.B_CoM_to_Foot_CurrentVel                    = FR_Foot.B_CurrentVel;
    RL_Foot.B_CoM_to_Foot_CurrentVel                    = RL_Foot.B_CurrentVel;
    RR_Foot.B_CoM_to_Foot_CurrentVel                    = RR_Foot.B_CurrentVel;
    
    vcopy(FL_Foot.B_Base2Foot_CurrentVel, 3, FL_Foot.B_CoM2Foot_CurrentVel);
    vcopy(FR_Foot.B_Base2Foot_CurrentVel, 3, FR_Foot.B_CoM2Foot_CurrentVel);
    vcopy(RL_Foot.B_Base2Foot_CurrentVel, 3, RL_Foot.B_CoM2Foot_CurrentVel);
    vcopy(RR_Foot.B_Base2Foot_CurrentVel, 3, RR_Foot.B_CoM2Foot_CurrentVel);  
    
    //* Current Base to Foot Velocity in Global Frame
    mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, FL_Foot.B_Base2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, FL_Foot.B_Base2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, FL_Foot.G_Base2Foot_CurrentVel);
    
    mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, FR_Foot.B_Base2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, FR_Foot.B_Base2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, FR_Foot.G_Base2Foot_CurrentVel);
    
    mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, RL_Foot.B_Base2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, RL_Foot.B_Base2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, RL_Foot.G_Base2Foot_CurrentVel);
    
    mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, RR_Foot.B_Base2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, RR_Foot.B_Base2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, RR_Foot.G_Base2Foot_CurrentVel);
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_Base_to_Foot_CurrentVel(AXIS)         = FL_Foot.G_Base2Foot_CurrentVel[AXIS + 1];
        FR_Foot.G_Base_to_Foot_CurrentVel(AXIS)         = FR_Foot.G_Base2Foot_CurrentVel[AXIS + 1];
        RL_Foot.G_Base_to_Foot_CurrentVel(AXIS)         = RL_Foot.G_Base2Foot_CurrentVel[AXIS + 1];
        RR_Foot.G_Base_to_Foot_CurrentVel(AXIS)         = RR_Foot.G_Base2Foot_CurrentVel[AXIS + 1];
    }    
    
    //* Current CoM to Foot Velocity in Global Frame
    
    //** B_w_gb_skew 가 아니라 G_w_gb_skew 여야할 것 같아서 바꿔둠
    // mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mmult(Current_C_gb, 3, 3, Current_G_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, FL_Foot.B_CoM2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, FL_Foot.B_CoM2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, FL_Foot.G_CoM2Foot_CurrentVel);
    
    // mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mmult(Current_C_gb, 3, 3, Current_G_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, FR_Foot.B_CoM2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, FR_Foot.B_CoM2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, FR_Foot.G_CoM2Foot_CurrentVel);
    
    // mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mmult(Current_C_gb, 3, 3, Current_G_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, RL_Foot.B_CoM2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, RL_Foot.B_CoM2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, RL_Foot.G_CoM2Foot_CurrentVel);
    
    // mmult(Current_C_gb, 3, 3, Current_B_w_gb_skew, 3, 3, tempOutMat);
    mmult(Current_C_gb, 3, 3, Current_G_w_gb_skew, 3, 3, tempOutMat);
    mvmult(tempOutMat, 3, 3, RR_Foot.B_CoM2Foot_CurrentPos, 3, tempOutVec);
    mvmult(Current_C_gb, 3, 3, RR_Foot.B_CoM2Foot_CurrentVel, 3, OutVec);
    vadd(tempOutVec, 3, OutVec, RR_Foot.G_CoM2Foot_CurrentVel);
    
    for (int AXIS = 0; AXIS < 3; ++AXIS) {
        FL_Foot.G_CoM_to_Foot_CurrentVel(AXIS)         = FL_Foot.G_CoM2Foot_CurrentVel[AXIS + 1];
        FR_Foot.G_CoM_to_Foot_CurrentVel(AXIS)         = FR_Foot.G_CoM2Foot_CurrentVel[AXIS + 1];
        RL_Foot.G_CoM_to_Foot_CurrentVel(AXIS)         = RL_Foot.G_CoM2Foot_CurrentVel[AXIS + 1];
        RR_Foot.G_CoM_to_Foot_CurrentVel(AXIS)         = RR_Foot.G_CoM2Foot_CurrentVel[AXIS + 1];
    }   
    
    base.G_Foot_to_Base_CurrentPos                  = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_Base_to_Foot_CurrentPos) + contact.Foot(FR_FootIndex)*(-FR_Foot.G_Base_to_Foot_CurrentPos) + contact.Foot(RL_FootIndex)*(-RL_Foot.G_Base_to_Foot_CurrentPos) + contact.Foot(RR_FootIndex)*(-RR_Foot.G_Base_to_Foot_CurrentPos))/ contact.Number;
    base.G_Foot_to_Base_CurrentVel                  = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_Base_to_Foot_CurrentVel) + contact.Foot(FR_FootIndex)*(-FR_Foot.G_Base_to_Foot_CurrentVel) + contact.Foot(RL_FootIndex)*(-RL_Foot.G_Base_to_Foot_CurrentVel) + contact.Foot(RR_FootIndex)*(-RR_Foot.G_Base_to_Foot_CurrentVel))/ contact.Number;
    
    com.G_Foot_to_CoM_CurrentPos                    = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_CoM_to_Foot_CurrentPos)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_CoM_to_Foot_CurrentPos)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_CoM_to_Foot_CurrentPos)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_CoM_to_Foot_CurrentPos))/ contact.Number;
    com.G_Foot_to_CoM_CurrentVel                    = (contact.Foot(FL_FootIndex)*(-FL_Foot.G_CoM_to_Foot_CurrentVel)  + contact.Foot(FR_FootIndex)*(-FR_Foot.G_CoM_to_Foot_CurrentVel)  + contact.Foot(RL_FootIndex)*(-RL_Foot.G_CoM_to_Foot_CurrentVel)  + contact.Foot(RR_FootIndex)*(-RR_Foot.G_CoM_to_Foot_CurrentVel))/ contact.Number;
    
    
    // Foot Contact Force Estimation
    
    
    free_matrix(OutMat, 1, 3, 1, 3);
    free_matrix(tempOutMat, 1, 3, 1, 3);
    free_vector(OutVec, 1, 3);
    free_vector(tempOutVec, 1, 3);

}


void CRobot::GenerateFriction() {
    static float JointVelThreshold     = 0.;
    static float sign                  = 1.;
    static double HR_weight             = 0.8;
    static double HP_weight             = 0.8;
    static double KN_weight             = 0.8;

    static float exp_func;
    static float exp_coeff             = 70.0;

    static double fric_real_coeff       = 1.0; 
    static double fric_virtual_coeff    = 0.0; 

    if(friction.Activeflag == false){
        for (int i = 0; i < JOINT_NUM; i++){
            friction.Current[i]    = 0;
        }
    }
    else{
		
        // FLHR
        if(joint[FLHR].RefVel > JointVelThreshold){sign = 1.;}
		else if(joint[FLHR].RefVel < -JointVelThreshold){sign = -1.;}
		else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLHR].RefVel));
		friction.Current[FLHR]      = fric_virtual_coeff* HR_weight*(friction.Viscous[FLHR] * joint[FLHR].RefVel + friction.Coulomb[FLHR]*sign);
        friction.Current[FLHR]      = friction.Current[FLHR] * exp_func;

        if(joint[FLHR].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FLHR].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLHR].CurrentVel));
        friction.Current[FLHR] += exp_func*fric_real_coeff*(HR_weight*(friction.Inertia[FLHR] * joint[FLHR].RefAcc + friction.Viscous[FLHR] * joint[FLHR].CurrentVel + friction.Coulomb[FLHR]*sign));

        // FRHR
		if(joint[FRHR].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[FRHR].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRHR].RefVel));
    	friction.Current[FRHR]     = fric_virtual_coeff* HR_weight*(friction.Viscous[FRHR] * joint[FRHR].RefVel + friction.Coulomb[FRHR]*sign);
        friction.Current[FRHR]      = friction.Current[FRHR] * exp_func;

        if(joint[FRHR].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FRHR].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRHR].CurrentVel));
        friction.Current[FRHR] += exp_func*fric_real_coeff*(HR_weight*(friction.Inertia[FRHR] * joint[FRHR].RefAcc + friction.Viscous[FRHR] * joint[FRHR].CurrentVel + friction.Coulomb[FRHR]*sign));

        // RLHR
        if(joint[RLHR].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RLHR].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLHR].RefVel));
    	friction.Current[RLHR]     = fric_virtual_coeff* HR_weight*(friction.Viscous[RLHR] * joint[RLHR].RefVel + friction.Coulomb[RLHR]*sign);
        friction.Current[RLHR]      = friction.Current[RLHR] * exp_func;
		

        if(joint[RLHR].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RLHR].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLHR].CurrentVel));
        friction.Current[RLHR] += exp_func*fric_real_coeff*(HR_weight*(friction.Inertia[RLHR] * joint[RLHR].RefAcc + friction.Viscous[RLHR] * joint[RLHR].CurrentVel + friction.Coulomb[RLHR]*sign));

        // RRHR
        if(joint[RRHR].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RRHR].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRHR].RefVel));
    	friction.Current[RRHR]     = fric_virtual_coeff* HR_weight*(friction.Viscous[RRHR] * joint[RRHR].RefVel + friction.Coulomb[RRHR]*sign);
        friction.Current[RRHR]      = friction.Current[RRHR] * exp_func;
		
        if(joint[RRHR].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RRHR].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRHR].CurrentVel));
        friction.Current[RRHR] += exp_func*fric_real_coeff*(HR_weight*(friction.Inertia[RRHR] * joint[RRHR].RefAcc + friction.Viscous[RRHR] * joint[RRHR].CurrentVel + friction.Coulomb[RRHR]*sign));

        // FLHP
        if(joint[FLHP].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[FLHP].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLHP].RefVel));
    	friction.Current[FLHP]     = fric_virtual_coeff* HP_weight*(friction.Viscous[FLHP] * joint[FLHP].RefVel + friction.Coulomb[FLHP]*sign);
        friction.Current[FLHP]      = friction.Current[FLHP] * exp_func;
		

        if(joint[FLHP].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FLHP].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLHP].CurrentVel));
        friction.Current[FLHP] += exp_func*fric_real_coeff*(HP_weight*(friction.Inertia[FLHP] * joint[FLHP].RefAcc + friction.Viscous[FLHP] * joint[FLHP].CurrentVel + friction.Coulomb[FLHP]*sign));

        // FRHP
        if(joint[FRHP].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[FRHP].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRHP].RefVel));
    	friction.Current[FRHP]     = fric_virtual_coeff* HP_weight*(friction.Viscous[FRHP] * joint[FRHP].RefVel + friction.Coulomb[FRHP]*sign);
        friction.Current[FRHP]      = friction.Current[FRHP] * exp_func;
		
        if(joint[FRHP].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FRHP].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRHP].CurrentVel));
        friction.Current[FRHP] += exp_func*fric_real_coeff*(HP_weight*(friction.Inertia[FRHP] * joint[FRHP].RefAcc + friction.Viscous[FRHP] * joint[FRHP].CurrentVel + friction.Coulomb[FRHP]*sign));


        // RLHP
        if(joint[RLHP].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RLHP].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLHP].RefVel));
    	friction.Current[RLHP]     = fric_virtual_coeff* HP_weight*(friction.Viscous[RLHP] * joint[RLHP].RefVel + friction.Coulomb[RLHP]*sign);
        friction.Current[RLHP]     = friction.Current[RLHP] * exp_func;
		

        if(joint[RLHP].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RLHP].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLHP].CurrentVel));
        friction.Current[RLHP] += exp_func*fric_real_coeff*(HP_weight*(friction.Inertia[RLHP] * joint[RLHP].RefAcc + friction.Viscous[RLHP] * joint[RLHP].CurrentVel + friction.Coulomb[RLHP]*sign));

        // RRHP
        if(joint[RRHP].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RRHP].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRHP].RefVel));
		friction.Current[RRHP]      = fric_virtual_coeff* HP_weight*(friction.Viscous[RRHP] * joint[RRHP].RefVel + friction.Coulomb[RRHP]*sign);
        friction.Current[RRHP]      = friction.Current[RRHP] * exp_func;
		

        if(joint[RRHP].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RRHP].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRHP].CurrentVel));
        friction.Current[RRHP] += exp_func*fric_real_coeff*(HP_weight*(friction.Inertia[RRHP] * joint[RRHP].RefAcc + friction.Viscous[RRHP] * joint[RRHP].CurrentVel + friction.Coulomb[RRHP]*sign));


        // FLKN
        if(joint[FLKN].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[FLKN].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLKN].RefVel));
		friction.Current[FLKN]     = fric_virtual_coeff* KN_weight*(friction.Viscous[FLKN] * joint[FLKN].RefVel + friction.Coulomb[FLKN]*sign);
        friction.Current[FLKN]     = friction.Current[FLKN] * exp_func;
		

        if(joint[FLKN].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FLKN].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FLKN].CurrentVel));
        friction.Current[FLKN] += exp_func*fric_real_coeff*(KN_weight*(friction.Inertia[FLKN] * joint[FLKN].RefAcc + friction.Viscous[FLKN] * joint[FLKN].CurrentVel + friction.Coulomb[FLKN]*sign));

        // FRKN
        if(joint[FRKN].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[FRKN].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRKN].RefVel));
    	friction.Current[FRKN]     = fric_virtual_coeff* KN_weight*(friction.Viscous[FRKN] * joint[FRKN].RefVel + friction.Coulomb[FRKN]*sign);
        friction.Current[FRKN]      = friction.Current[FRKN] * exp_func;
		
        if(joint[FRKN].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[FRKN].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[FRKN].CurrentVel));
        friction.Current[FRKN] += exp_func*fric_real_coeff*(KN_weight*(friction.Inertia[FRKN] * joint[FRKN].RefAcc + friction.Viscous[FRKN] * joint[FRKN].CurrentVel + friction.Coulomb[FRKN]*sign));

        // RLKN
        if(joint[RLKN].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RLKN].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLKN].RefVel));
    	friction.Current[RLKN]     = fric_virtual_coeff* KN_weight*(friction.Viscous[RLKN] * joint[RLKN].RefVel + friction.Coulomb[RLKN]*sign);
        friction.Current[RLKN]      = friction.Current[RLKN] * exp_func;
		
        if(joint[RLKN].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RLKN].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RLKN].CurrentVel));
        friction.Current[RLKN] += exp_func*fric_real_coeff*(KN_weight*(friction.Inertia[RLKN] * joint[RLKN].RefAcc + friction.Viscous[RLKN] * joint[RLKN].CurrentVel + friction.Coulomb[RLKN]*sign));

        // RRKN
        if(joint[RRKN].RefVel > JointVelThreshold){sign = 1;}
		else if(joint[RRKN].RefVel < -JointVelThreshold){sign = -1;}
		else{sign = 0;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRKN].RefVel));
    	friction.Current[RRKN]     = fric_virtual_coeff* KN_weight*(friction.Viscous[RRKN] * joint[RRKN].RefVel + friction.Coulomb[RRKN]*sign);
        friction.Current[RRKN]      = friction.Current[RRKN] * exp_func;

        if(joint[RRKN].CurrentVel > JointVelThreshold){sign = 1.;}
        else if(joint[RRKN].CurrentVel < -JointVelThreshold){sign = -1.;}
        else{sign = 0.;}
        exp_func = 1 - exp(-exp_coeff*abs(joint[RRKN].CurrentVel));
        friction.Current[RRKN] += exp_func*fric_real_coeff*(KN_weight*(friction.Inertia[RRKN] * joint[RRKN].RefAcc + friction.Viscous[RRKN] * joint[RRKN].CurrentVel + friction.Coulomb[RRKN]*sign));
    }
}


void CRobot::ComputeTorqueControl() {
    /* ComputeTorqueControl
     * Compute Joint Torque Control
     */ 
    VectorXf EP_RefPos                  = VectorXf::Zero(12);
    VectorXf EP_RefVel                  = VectorXf::Zero(12);
    VectorXf EP_CurrentPos              = VectorXf::Zero(12);
    VectorXf EP_CurrrentVel             = VectorXf::Zero(12);
    
    EP_RefPos                           <<        FL_Foot.B_RefPos,           FR_Foot.B_RefPos,           RL_Foot.B_RefPos,            RR_Foot.B_RefPos;
    EP_RefVel                           <<        FL_Foot.B_RefVel,           FR_Foot.B_RefVel,           RL_Foot.B_RefVel,            RR_Foot.B_RefVel;
    EP_CurrentPos                       <<    FL_Foot.B_CurrentPos,       FR_Foot.B_CurrentPos,       RL_Foot.B_CurrentPos,        RR_Foot.B_CurrentPos;
    EP_CurrrentVel                      <<    FL_Foot.B_CurrentVel,       FR_Foot.B_CurrentVel,       RL_Foot.B_CurrentVel,        RR_Foot.B_CurrentVel;

    for (int index = 0; index < joint_DoF; ++index) {
        torque.task[index + 6]      = gain.TaskKp[index]  * (EP_RefPos[index] - EP_CurrentPos[index])       + gain.TaskKd[index]*(EP_RefVel[index] - EP_CurrrentVel[index]);
        torque.joint[index + 6]     = gain.JointKp[index] * (joint[index].RefPos - joint[index].CurrentPos) + gain.JointKd[index]*(joint[index].RefVel - joint[index].CurrentVel);
    }
    
    for (int index = 0; index < joint_DoF + 6; ++index) {
        
        if(index < 3){
            JointRefAcc[index]      = base.G_RefAcc(index)              + gain.BasePosKd[index]*(base.G_Foot_to_Base_RefVel(index)      - base.G_Foot_to_Base_CurrentVel(index))        + gain.BasePosKp[index]*(base.G_Foot_to_Base_RefPos(index)      - base.G_Foot_to_Base_CurrentPos(index));            
        }
        else if(index < 6){            
            JointRefAcc[index]      = base.B_RefAngularAcc(index - 3)   + gain.BasePosKd[index]*(base.B_RefAngularVel(index - 3)        - base.B_CurrentAngularVel(index - 3))          + gain.BasePosKp[index]*(base.G_RefOri(index - 3)               - base.G_CurrentOri(index - 3));
        }
        else{
            JointRefAcc[index]      = joint[index - 6].RefAcc + (torque.joint.cast <double> ())[index];
        }
        
    }
    
     //* NonlinearEffects(RBDL)
    CompositeRigidBodyAlgorithm(*m_pModel, RobotState, M_term, true);
    
    // M_term_hat = M_term;
    
    for (int index = 6; index < joint_DoF + 6; ++index) {
        M_term_hat(index, index)        = M_term(index, index);
    }

    NonlinearEffects(*m_pModel, RobotState, RobotStatedot, hatNonLinearEffects);          // C_term + G_term
    NonlinearEffects(*m_pModel, RobotState, VectorNd::Zero(m_pModel->dof_count), G_term); // G_term
    C_term          = hatNonLinearEffects - G_term;
    
    M_term_Torque   = M_term_hat*(JointRefAcc);
            
    CTC_Torque = (M_term_Torque + C_term + G_term_Weight*G_term + osqp.torque); // 230704 까지 사용중이던 토크
    //CTC_Torque = (C_term + G_term_Weight*G_term + osqp.torque); // M_term 없애봄>
    
    //CTC_Torque = osqp.torque;



//    CTC_Torque = (osqp.torque + C_term + G_term_Weight*G_term + (torque.joint).cast <double> ()); // 아님
    
    
    for (int nJoint = 0; nJoint < joint_DoF; nJoint++) {
        joint[nJoint].RefTorque = CTC_Torque(6 + nJoint);
    }
}

Vector3f CRobot::GetCOM(Vector3f BasePos, Vector3f BaseOri, Vector12f JointAngle) {
    /* GetCOM
     * input : BasePos, BaseOri, JointAngle
     * output : Calculate Robot Total Center of Mass
     * CoM Position : CoM position measured from each frame.
     */
    
    const float m_BODY                              = BodyKine.m_body_link;
    
    const float m_FL_HIP                            = BodyKine.m_FL_hip_link;
    const float m_FL_THIGH                          = BodyKine.m_FL_thigh_link;
    const float m_FL_CALF                           = BodyKine.m_FL_calf_link;
    const float m_FL_TIP                            = BodyKine.m_FL_tip_link;
    const float m_FL_LEG                            = m_FL_HIP + m_FL_THIGH + m_FL_CALF + m_FL_TIP;
    
    const float m_FR_HIP                            = BodyKine.m_FR_hip_link;
    const float m_FR_THIGH                          = BodyKine.m_FR_thigh_link;
    const float m_FR_CALF                           = BodyKine.m_FR_calf_link;
    const float m_FR_TIP                            = BodyKine.m_FR_tip_link;
    const float m_FR_LEG                            = m_FR_HIP + m_FR_THIGH + m_FR_CALF + m_FR_TIP;
    
    const float m_RL_HIP                            = BodyKine.m_RL_hip_link;
    const float m_RL_THIGH                          = BodyKine.m_RL_thigh_link;
    const float m_RL_CALF                           = BodyKine.m_RL_calf_link;
    const float m_RL_TIP                            = BodyKine.m_RL_tip_link;
    const float m_RL_LEG                            = m_RL_HIP + m_RL_THIGH + m_RL_CALF + m_RL_TIP;
    
    const float m_RR_HIP                            = BodyKine.m_RR_hip_link;
    const float m_RR_THIGH                          = BodyKine.m_RR_thigh_link;
    const float m_RR_CALF                           = BodyKine.m_RR_calf_link;
    const float m_RR_TIP                            = BodyKine.m_RR_tip_link;
    const float m_RR_LEG                            = m_RR_HIP + m_RR_THIGH + m_RR_CALF + m_RR_TIP;
    
    
    
    const float m_ROBOT                             = (m_FL_LEG + m_FR_LEG + m_RL_LEG + m_RR_LEG) + m_BODY;
    
    Vector4f POS_CoM_BODY_FROM_BASE;

    Vector4f POS_CoM_FL_HIP_FROM_FL_HIP;
    Vector4f POS_CoM_FL_THIGH_FROM_FL_THIGH;
    Vector4f POS_CoM_FL_CALF_FROM_FL_CALF;
    Vector4f POS_CoM_FL_TIP_FROM_FL_TIP;

    Vector4f POS_CoM_FR_HIP_FROM_FR_HIP;
    Vector4f POS_CoM_FR_THIGH_FROM_FR_THIGH;
    Vector4f POS_CoM_FR_CALF_FROM_FR_CALF;
    Vector4f POS_CoM_FR_TIP_FROM_FR_TIP;
    
    Vector4f POS_CoM_RL_HIP_FROM_RL_HIP;
    Vector4f POS_CoM_RL_THIGH_FROM_RL_THIGH;
    Vector4f POS_CoM_RL_CALF_FROM_RL_CALF;
    Vector4f POS_CoM_RL_TIP_FROM_RL_TIP;

    Vector4f POS_CoM_RR_HIP_FROM_RR_HIP;
    Vector4f POS_CoM_RR_THIGH_FROM_RR_THIGH;
    Vector4f POS_CoM_RR_CALF_FROM_RR_CALF;
    Vector4f POS_CoM_RR_TIP_FROM_RR_TIP;

    MatrixXf ROT_WORLD2BASE                         = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_WORLD2BASE                       = MatrixXf::Identity(4, 4);
    
    MatrixXf TRANS_FL_BASE2HIP                      = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FL_HIP2THIGH                     = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FL_THIGH2CALF                    = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FL_CALF2TIP                      = MatrixXf::Identity(4, 4);
    
    MatrixXf TRANS_FR_BASE2HIP                      = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FR_HIP2THIGH                     = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FR_THIGH2CALF                    = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_FR_CALF2TIP                      = MatrixXf::Identity(4, 4);
    
    MatrixXf TRANS_RL_BASE2HIP                      = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RL_HIP2THIGH                     = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RL_THIGH2CALF                    = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RL_CALF2TIP                      = MatrixXf::Identity(4, 4);

    MatrixXf TRANS_RR_BASE2HIP                      = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RR_HIP2THIGH                     = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RR_THIGH2CALF                    = MatrixXf::Identity(4, 4);
    MatrixXf TRANS_RR_CALF2TIP                      = MatrixXf::Identity(4, 4);
    
    Vector4f POS_CoM_FL_HIP_FROM_BASE;
    Vector4f POS_CoM_FL_THIGH_FROM_BASE;
    Vector4f POS_CoM_FL_CALF_FROM_BASE;
    Vector4f POS_CoM_FL_TIP_FROM_BASE;
    Vector4f POS_CoM_FL_LINKS_FROM_BASE;

    Vector4f POS_CoM_FR_HIP_FROM_BASE;
    Vector4f POS_CoM_FR_THIGH_FROM_BASE;
    Vector4f POS_CoM_FR_CALF_FROM_BASE;
    Vector4f POS_CoM_FR_TIP_FROM_BASE;
    Vector4f POS_CoM_FR_LINKS_FROM_BASE;
    
    Vector4f POS_CoM_RL_HIP_FROM_BASE;
    Vector4f POS_CoM_RL_THIGH_FROM_BASE;
    Vector4f POS_CoM_RL_CALF_FROM_BASE;
    Vector4f POS_CoM_RL_TIP_FROM_BASE;
    Vector4f POS_CoM_RL_LINKS_FROM_BASE;

    Vector4f POS_CoM_RR_HIP_FROM_BASE;
    Vector4f POS_CoM_RR_THIGH_FROM_BASE;
    Vector4f POS_CoM_RR_CALF_FROM_BASE;
    Vector4f POS_CoM_RR_TIP_FROM_BASE;
    Vector4f POS_CoM_RR_LINKS_FROM_BASE;

    Vector4f POS_CoM_ROBOT_FROM_BASE;
    Vector4f POS_CoM_ROBOT_FROM_WORLD;
    Vector3f POS_CoM_ROBOT;

    // LINK CoM Position
    POS_CoM_BODY_FROM_BASE                      <<        BodyKine.c_body_link(AXIS_X),       BodyKine.c_body_link(AXIS_Y),       BodyKine.c_body_link(AXIS_Z),  1;

    POS_CoM_FL_HIP_FROM_FL_HIP                  <<      BodyKine.c_FL_hip_link(AXIS_X),     BodyKine.c_FL_hip_link(AXIS_Y),     BodyKine.c_FL_hip_link(AXIS_Z),  1;
    POS_CoM_FL_THIGH_FROM_FL_THIGH              <<    BodyKine.c_FL_thigh_link(AXIS_X),   BodyKine.c_FL_thigh_link(AXIS_Y),   BodyKine.c_FL_thigh_link(AXIS_Z),  1;
    
    if(Type == LEGMODE){
        POS_CoM_FL_CALF_FROM_FL_CALF            <<   0.000133,    0.003337,   -0.081880, 1;
        POS_CoM_FL_TIP_FROM_FL_TIP              <<   0.000494,           0,    0.002943, 1;
    }
    else if(Type == WHEELMODE){
        POS_CoM_FL_CALF_FROM_FL_CALF            <<  -0.006691,    0.000525,    -0.14037, 1;
        POS_CoM_FL_TIP_FROM_FL_TIP              <<        0.0,    0.040126,           0, 1;
    }
            
    POS_CoM_FR_HIP_FROM_FR_HIP                  <<      BodyKine.c_FR_hip_link(AXIS_X),     BodyKine.c_FR_hip_link(AXIS_Y),     BodyKine.c_FR_hip_link(AXIS_Z),  1;
    POS_CoM_FR_THIGH_FROM_FR_THIGH              <<    BodyKine.c_FR_thigh_link(AXIS_X),   BodyKine.c_FR_thigh_link(AXIS_Y),   BodyKine.c_FR_thigh_link(AXIS_Z),  1;
    
    if(Type == LEGMODE){
        POS_CoM_FR_CALF_FROM_FR_CALF                <<   0.000137,   -0.003337,   -0.081883, 1;
        POS_CoM_FR_TIP_FROM_FR_TIP                  <<   0.000494,           0,    0.002943, 1;
    }
    else if(Type == WHEELMODE){
        POS_CoM_FR_CALF_FROM_FR_CALF                <<  -0.006688,   -0.000525,    -0.14037, 1;
        POS_CoM_FR_TIP_FROM_FR_TIP                  <<          0,   -0.040126,           0, 1;
    }
    
    POS_CoM_RL_HIP_FROM_RL_HIP                  <<      BodyKine.c_RL_hip_link(AXIS_X),     BodyKine.c_RL_hip_link(AXIS_Y),     BodyKine.c_RL_hip_link(AXIS_Z),  1;
    POS_CoM_RL_THIGH_FROM_RL_THIGH              <<    BodyKine.c_RL_thigh_link(AXIS_X),   BodyKine.c_RL_thigh_link(AXIS_Y),   BodyKine.c_RL_thigh_link(AXIS_Z),  1;
    
    cout << "POS_CoM_RL_HIP_FROM_RL_HIP : "         << POS_CoM_RL_HIP_FROM_RL_HIP << endl;   
    cout << "POS_CoM_RL_THIGH_FROM_RL_THIGH : "     << POS_CoM_RL_THIGH_FROM_RL_THIGH << endl;   
    
    if(Type == LEGMODE){
        POS_CoM_RL_CALF_FROM_RL_CALF                <<   0.000133,    0.003337,   -0.081880, 1;
        POS_CoM_RL_TIP_FROM_RL_TIP                  <<   0.000494,           0,    0.002943, 1;
    }
    else if(Type == WHEELMODE){
        POS_CoM_RL_CALF_FROM_RL_CALF                <<   -0.00669,    0.000525,    -0.14037, 1;
        POS_CoM_RL_TIP_FROM_RL_TIP                  <<          0,    0.040126,           0, 1;
    }
            
    POS_CoM_RR_HIP_FROM_RR_HIP                  <<      BodyKine.c_RR_hip_link(AXIS_X),     BodyKine.c_RR_hip_link(AXIS_Y),     BodyKine.c_RR_hip_link(AXIS_Z),  1;
    POS_CoM_RR_THIGH_FROM_RR_THIGH              <<    BodyKine.c_RR_thigh_link(AXIS_X),   BodyKine.c_RR_thigh_link(AXIS_Y),   BodyKine.c_RR_thigh_link(AXIS_Z),  1;
    
    if(Type == LEGMODE){
        POS_CoM_RR_CALF_FROM_RR_CALF                <<   0.000137,   -0.003337,   -0.081883, 1;
        POS_CoM_RR_TIP_FROM_RR_TIP                  <<   0.000494,           0,    0.002943, 1;
    }
    else if(Type == WHEELMODE){
        POS_CoM_RR_CALF_FROM_RR_CALF                <<  -0.006688,   -0.000525,    -0.14037, 1;
        POS_CoM_RR_TIP_FROM_RR_TIP                  <<          0,   -0.040126,           0, 1;
    }

    // TRANSFORMATION & ROTATION MATRIX
    ROT_WORLD2BASE      = Z_Rot(BaseOri(AXIS_YAW),4) * Y_Rot(BaseOri(AXIS_PITCH),4) * X_Rot(BaseOri(AXIS_ROLL),4);

    TRANS_WORLD2BASE    <<                          1,                          0,                          0,                    BasePos(AXIS_X),
                                                    0,                          1,                          0,                    BasePos(AXIS_Y),
                                                    0,                          0,                          1,                    BasePos(AXIS_Z),
                                                    0,                          0,                          0,                                  1;

        // FL LINKS
    TRANS_FL_BASE2HIP   <<                          1,                          0,                          0,      lowerbody.BASE_TO_HR_LENGTH(AXIS_X),
                                                    0,      cos(JointAngle(FLHR)),     -sin(JointAngle(FLHR)),      lowerbody.BASE_TO_HR_LENGTH(AXIS_Y),
                                                    0,      sin(JointAngle(FLHR)),      cos(JointAngle(FLHR)),     -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_FL_HIP2THIGH  <<      cos(JointAngle(FLHP)),                          0,      sin(JointAngle(FLHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.HR_TO_HP_LENGTH(AXIS_Y),
                               -sin(JointAngle(FLHP)),                          0,      cos(JointAngle(FLHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_FL_THIGH2CALF <<      cos(JointAngle(FLKN)),                          0,      sin(JointAngle(FLKN)),        lowerbody.HP_TO_KN_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.HP_TO_KN_LENGTH(AXIS_Y),
                               -sin(JointAngle(FLKN)),                          0,      cos(JointAngle(FLKN)),       -lowerbody.HP_TO_KN_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_FL_CALF2TIP   <<                          1,                          0,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_Y),
                                                    0,                          0,                          1,       -lowerbody.KN_TO_EP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                  1; 
    
    POS_CoM_FL_HIP_FROM_BASE    = TRANS_FL_BASE2HIP * POS_CoM_FL_HIP_FROM_FL_HIP;
    POS_CoM_FL_THIGH_FROM_BASE  = TRANS_FL_BASE2HIP * TRANS_FL_HIP2THIGH * POS_CoM_FL_THIGH_FROM_FL_THIGH;
    POS_CoM_FL_CALF_FROM_BASE   = TRANS_FL_BASE2HIP * TRANS_FL_HIP2THIGH * TRANS_FL_THIGH2CALF * POS_CoM_FL_CALF_FROM_FL_CALF;
    POS_CoM_FL_TIP_FROM_BASE    = TRANS_FL_BASE2HIP * TRANS_FL_HIP2THIGH * TRANS_FL_THIGH2CALF * TRANS_FL_CALF2TIP * POS_CoM_FL_TIP_FROM_FL_TIP;
    POS_CoM_FL_LINKS_FROM_BASE  = (m_FL_HIP * POS_CoM_FL_HIP_FROM_BASE + m_FL_THIGH * POS_CoM_FL_THIGH_FROM_BASE + m_FL_CALF * POS_CoM_FL_CALF_FROM_BASE + m_FL_TIP * POS_CoM_FL_TIP_FROM_BASE) / (m_FL_LEG);
    
    // FR LINKS
    TRANS_FR_BASE2HIP   <<                          1,                          0,                          0,       lowerbody.BASE_TO_HR_LENGTH(AXIS_X),
                                                    0,      cos(JointAngle(FRHR)),     -sin(JointAngle(FRHR)),      -lowerbody.BASE_TO_HR_LENGTH(AXIS_Y),
                                                    0,      sin(JointAngle(FRHR)),      cos(JointAngle(FRHR)),      -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                         1;

    TRANS_FR_HIP2THIGH  <<      cos(JointAngle(FRHP)),                          0,      sin(JointAngle(FRHP)),         lowerbody.HR_TO_HP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        -lowerbody.HR_TO_HP_LENGTH(AXIS_Y),
                               -sin(JointAngle(FRHP)),                          0,      cos(JointAngle(FRHP)),         lowerbody.HR_TO_HP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                         1;

    TRANS_FR_THIGH2CALF <<      cos(JointAngle(FRKN)),                          0,      sin(JointAngle(FRKN)),         lowerbody.HP_TO_KN_LENGTH(AXIS_X),
                                                    0,                          1,                          0,         lowerbody.HP_TO_KN_LENGTH(AXIS_Y),
                               -sin(JointAngle(FRKN)),                          0,      cos(JointAngle(FRKN)),        -lowerbody.HP_TO_KN_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                         1;

    TRANS_FR_CALF2TIP   <<                          1,                          0,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_Y),
                                                    0,                          0,                          1,       -lowerbody.KN_TO_EP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1; 
    
    POS_CoM_FR_HIP_FROM_BASE    = TRANS_FR_BASE2HIP * POS_CoM_FR_HIP_FROM_FR_HIP;
    POS_CoM_FR_THIGH_FROM_BASE  = TRANS_FR_BASE2HIP * TRANS_FR_HIP2THIGH * POS_CoM_FR_THIGH_FROM_FR_THIGH;
    POS_CoM_FR_CALF_FROM_BASE   = TRANS_FR_BASE2HIP * TRANS_FR_HIP2THIGH * TRANS_FR_THIGH2CALF * POS_CoM_FR_CALF_FROM_FR_CALF;
    POS_CoM_FR_TIP_FROM_BASE    = TRANS_FR_BASE2HIP * TRANS_FR_HIP2THIGH * TRANS_FR_THIGH2CALF * TRANS_FR_CALF2TIP * POS_CoM_FR_TIP_FROM_FR_TIP;
    POS_CoM_FR_LINKS_FROM_BASE  = (m_FR_HIP * POS_CoM_FR_HIP_FROM_BASE + m_FR_THIGH * POS_CoM_FR_THIGH_FROM_BASE + m_FR_CALF * POS_CoM_FR_CALF_FROM_BASE + m_FR_TIP * POS_CoM_FR_TIP_FROM_BASE) / (m_FR_LEG);
    
    // RL LINKS
    TRANS_RL_BASE2HIP   <<                          1,                          0,                          0,     -lowerbody.BASE_TO_HR_LENGTH(AXIS_X),
                                                    0,      cos(JointAngle(RLHR)),     -sin(JointAngle(RLHR)),      lowerbody.BASE_TO_HR_LENGTH(AXIS_Y),
                                                    0,      sin(JointAngle(RLHR)),      cos(JointAngle(RLHR)),     -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_RL_HIP2THIGH  <<      cos(JointAngle(RLHP)),                          0,      sin(JointAngle(RLHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.HR_TO_HP_LENGTH(AXIS_Y),
                               -sin(JointAngle(RLHP)),                          0,      cos(JointAngle(RLHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;
 
    TRANS_RL_THIGH2CALF <<      cos(JointAngle(RLKN)),                          0,      sin(JointAngle(RLKN)),        lowerbody.HP_TO_KN_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.HP_TO_KN_LENGTH(AXIS_Y),
                               -sin(JointAngle(RLKN)),                          0,      cos(JointAngle(RLKN)),       -lowerbody.HP_TO_KN_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;
    
    TRANS_RL_CALF2TIP   <<                          1,                          0,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_Y),
                                                    0,                          0,                          1,       -lowerbody.KN_TO_EP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1; 

    POS_CoM_RL_HIP_FROM_BASE    = TRANS_RL_BASE2HIP * POS_CoM_RL_HIP_FROM_RL_HIP;
    POS_CoM_RL_THIGH_FROM_BASE  = TRANS_RL_BASE2HIP * TRANS_RL_HIP2THIGH * POS_CoM_RL_THIGH_FROM_RL_THIGH;
    POS_CoM_RL_CALF_FROM_BASE   = TRANS_RL_BASE2HIP * TRANS_RL_HIP2THIGH * TRANS_RL_THIGH2CALF * POS_CoM_RL_CALF_FROM_RL_CALF;
    POS_CoM_RL_TIP_FROM_BASE    = TRANS_RL_BASE2HIP * TRANS_RL_HIP2THIGH * TRANS_RL_THIGH2CALF * TRANS_RL_CALF2TIP * POS_CoM_RL_TIP_FROM_RL_TIP;
    POS_CoM_RL_LINKS_FROM_BASE  = (m_RL_HIP * POS_CoM_RL_HIP_FROM_BASE + m_RL_THIGH * POS_CoM_RL_THIGH_FROM_BASE + m_RL_CALF * POS_CoM_RL_CALF_FROM_BASE + m_RL_TIP * POS_CoM_RL_TIP_FROM_BASE) / (m_RL_LEG);

    // RR LINKS
    TRANS_RR_BASE2HIP   <<                          1,                          0,                          0,     -lowerbody.BASE_TO_HR_LENGTH(AXIS_X),
                                                    0,      cos(JointAngle(RRHR)),     -sin(JointAngle(RRHR)),     -lowerbody.BASE_TO_HR_LENGTH(AXIS_Y),
                                                    0,      sin(JointAngle(RRHR)),      cos(JointAngle(RRHR)),     -lowerbody.BASE_TO_HR_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_RR_HIP2THIGH  <<      cos(JointAngle(RRHP)),                          0,      sin(JointAngle(RRHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,       -lowerbody.HR_TO_HP_LENGTH(AXIS_Y),
                               -sin(JointAngle(RRHP)),                          0,      cos(JointAngle(RRHP)),        lowerbody.HR_TO_HP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;

    TRANS_RR_THIGH2CALF <<      cos(JointAngle(RRKN)),                          0,      sin(JointAngle(RRKN)),        lowerbody.HP_TO_KN_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.HP_TO_KN_LENGTH(AXIS_Y),
                               -sin(JointAngle(RRKN)),                          0,      cos(JointAngle(RRKN)),       -lowerbody.HP_TO_KN_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1;
    
    TRANS_RR_CALF2TIP   <<                          1,                          0,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_X),
                                                    0,                          1,                          0,        lowerbody.KN_TO_EP_LENGTH(AXIS_Y),
                                                    0,                          0,                          1,       -lowerbody.KN_TO_EP_LENGTH(AXIS_Z),
                                                    0,                          0,                          0,                                        1; 

    POS_CoM_RR_HIP_FROM_BASE    = TRANS_RR_BASE2HIP * POS_CoM_RR_HIP_FROM_RR_HIP;
    POS_CoM_RR_THIGH_FROM_BASE  = TRANS_RR_BASE2HIP * TRANS_RR_HIP2THIGH * POS_CoM_RR_THIGH_FROM_RR_THIGH;
    POS_CoM_RR_CALF_FROM_BASE   = TRANS_RR_BASE2HIP * TRANS_RR_HIP2THIGH * TRANS_RR_THIGH2CALF * POS_CoM_RR_CALF_FROM_RR_CALF;
    POS_CoM_RR_TIP_FROM_BASE    = TRANS_RR_BASE2HIP * TRANS_RR_HIP2THIGH * TRANS_RR_THIGH2CALF * TRANS_RR_CALF2TIP * POS_CoM_RR_TIP_FROM_RR_TIP;
    POS_CoM_RR_LINKS_FROM_BASE  = (m_RR_HIP * POS_CoM_RR_HIP_FROM_BASE + m_RR_THIGH * POS_CoM_RR_THIGH_FROM_BASE + m_RR_CALF * POS_CoM_RR_CALF_FROM_BASE + m_RR_TIP * POS_CoM_RR_TIP_FROM_BASE) / (m_RR_LEG);

    // Robot CoM    
    
    POS_CoM_ROBOT_FROM_BASE     =   ((m_BODY * POS_CoM_BODY_FROM_BASE) + (m_FL_LEG * POS_CoM_FL_LINKS_FROM_BASE) + (m_FR_LEG * POS_CoM_FR_LINKS_FROM_BASE) + (m_RL_LEG * POS_CoM_RL_LINKS_FROM_BASE) + (m_RR_LEG * POS_CoM_RR_LINKS_FROM_BASE)) / (m_ROBOT);
    
    POS_CoM_ROBOT_FROM_WORLD    =   TRANS_WORLD2BASE * ROT_WORLD2BASE * POS_CoM_ROBOT_FROM_BASE;
    POS_CoM_ROBOT               =   POS_CoM_ROBOT_FROM_WORLD.block<3, 1>(0, 0);

    return POS_CoM_ROBOT;
}

void CRobot::SaveData() {
    /* SaveData
     * Save Robot Data
     */

    for (int nJoint = 0; nJoint < JOINT_NUM; nJoint++) {
        print.Data(nJoint + 1)     = (osqp.Ref_GRF)(nJoint);
    }

    // Foot->CoM Position in Global Frame
    print.Data(13)                  = com.G_Foot_to_CoM_RefPos(AXIS_X);
    print.Data(14)                  = com.G_Foot_to_CoM_RefPos(AXIS_Y);
    print.Data(15)                  = com.G_Foot_to_CoM_RefPos(AXIS_Z);
    
    print.Data(16)                  = com.G_Foot_to_CoM_CurrentPos(AXIS_X);
    print.Data(17)                  = com.G_Foot_to_CoM_CurrentPos(AXIS_Y);
    print.Data(18)                  = com.G_Foot_to_CoM_CurrentPos(AXIS_Z);

    // print.Data(16)                  = base.G_Foot_to_Base_CurrentPos(AXIS_X);
    // print.Data(17)                  = base.G_Foot_to_Base_CurrentPos(AXIS_Y);
    // print.Data(18)                  = base.G_Foot_to_Base_CurrentPos(AXIS_Z);

    // Foot->CoM Velocity in Global Frame
    print.Data(19)                  = com.G_Foot_to_CoM_RefVel(AXIS_X);
    print.Data(20)                  = com.G_Foot_to_CoM_RefVel(AXIS_Y);
    print.Data(21)                  = com.G_Foot_to_CoM_RefVel(AXIS_Z);

    print.Data(22)                  = com.G_Foot_to_CoM_CurrentVel(AXIS_X);
    print.Data(23)                  = com.G_Foot_to_CoM_CurrentVel(AXIS_Y);
    print.Data(24)                  = com.G_Foot_to_CoM_CurrentVel(AXIS_Z);

    // print.Data(22)                  = base.G_Foot_to_Base_CurrentVel(AXIS_X);
    // print.Data(23)                  = base.G_Foot_to_Base_CurrentVel(AXIS_Y);
    // print.Data(24)                  = base.G_Foot_to_Base_CurrentVel(AXIS_Z);


    // CoM Pos/Vel in Global Frame
    print.Data(25)                  = com.G_RefPos(AXIS_X);
    print.Data(26)                  = com.G_RefPos(AXIS_Y);
    print.Data(27)                  = com.G_RefPos(AXIS_Z);
    
    print.Data(28)                  = com.G_RefVel(AXIS_X);
    print.Data(29)                  = com.G_RefVel(AXIS_Y);
    print.Data(30)                  = com.G_RefVel(AXIS_Z);

    // Base Pos/Vel in Global Frame
    print.Data(31)                  = base.G_RefPos(AXIS_X);
    print.Data(32)                  = base.G_RefPos(AXIS_Y);
    print.Data(33)                  = base.G_RefPos(AXIS_Z);
    
    print.Data(34)                  = base.G_RefVel(AXIS_X);
    print.Data(35)                  = base.G_RefVel(AXIS_Y);
    print.Data(36)                  = base.G_RefVel(AXIS_Z);
    
    // Base Orientation & AngularVelocity
    print.Data(37)                  = base.G_RefOri(AXIS_ROLL);
    print.Data(38)                  = base.G_RefOri(AXIS_PITCH);
    print.Data(39)                  = base.G_RefOri(AXIS_YAW);

    print.Data(40)                  = base.G_CurrentOri(AXIS_ROLL);
    print.Data(41)                  = base.G_CurrentOri(AXIS_PITCH);
    print.Data(42)                  = base.G_CurrentOri(AXIS_YAW);
    
    print.Data(43)                  = base.G_RefAngularVel(AXIS_ROLL);
    print.Data(44)                  = base.G_RefAngularVel(AXIS_PITCH);
    print.Data(45)                  = base.G_RefAngularVel(AXIS_YAW);
    
    print.Data(46)                  = base.G_CurrentAngularVel(AXIS_ROLL);
    print.Data(47)                  = base.G_CurrentAngularVel(AXIS_PITCH);
    print.Data(48)                  = base.G_CurrentAngularVel(AXIS_YAW);

    print.Data(49)                  = base.B_CurrentAngularVel(AXIS_ROLL);
    print.Data(50)                  = base.B_CurrentAngularVel(AXIS_PITCH);
    print.Data(51)                  = base.B_CurrentAngularVel(AXIS_YAW);

    // Foot Pos in Global Frame
    print.Data(52)                  = FL_Foot.G_RefPos(AXIS_X);
    print.Data(53)                  = FL_Foot.G_RefPos(AXIS_Y);
    print.Data(54)                  = FL_Foot.G_RefPos(AXIS_Z);

    print.Data(55)                  = FR_Foot.G_RefPos(AXIS_X);
    print.Data(56)                  = FR_Foot.G_RefPos(AXIS_Y);
    print.Data(57)                  = FR_Foot.G_RefPos(AXIS_Z);

    print.Data(58)                  = RL_Foot.G_RefPos(AXIS_X);
    print.Data(59)                  = RL_Foot.G_RefPos(AXIS_Y);
    print.Data(60)                  = RL_Foot.G_RefPos(AXIS_Z);

    print.Data(61)                  = RR_Foot.G_RefPos(AXIS_X);
    print.Data(62)                  = RR_Foot.G_RefPos(AXIS_Y);
    print.Data(63)                  = RR_Foot.G_RefPos(AXIS_Z);

    // Foot Vel in Global Frame
    print.Data(64)                  = FL_Foot.G_RefVel(AXIS_X);
    print.Data(65)                  = FL_Foot.G_RefVel(AXIS_Y);
    print.Data(66)                  = FL_Foot.G_RefVel(AXIS_Z);

    print.Data(67)                  = FR_Foot.G_RefVel(AXIS_X);
    print.Data(68)                  = FR_Foot.G_RefVel(AXIS_Y);
    print.Data(69)                  = FR_Foot.G_RefVel(AXIS_Z);

    print.Data(70)                  = RL_Foot.G_RefVel(AXIS_X);
    print.Data(71)                  = RL_Foot.G_RefVel(AXIS_Y);
    print.Data(72)                  = RL_Foot.G_RefVel(AXIS_Z);

    print.Data(73)                  = RR_Foot.G_RefVel(AXIS_X);
    print.Data(74)                  = RR_Foot.G_RefVel(AXIS_Y);
    print.Data(75)                  = RR_Foot.G_RefVel(AXIS_Z);

    // Foot Pos in Base Frame 
    print.Data(76)                  = FL_Foot.B_RefPos(AXIS_X);
    print.Data(77)                  = FL_Foot.B_RefPos(AXIS_Y);
    print.Data(78)                  = FL_Foot.B_RefPos(AXIS_Z);
    
    print.Data(79)                  = FR_Foot.B_RefPos(AXIS_X);
    print.Data(80)                  = FR_Foot.B_RefPos(AXIS_Y);
    print.Data(81)                  = FR_Foot.B_RefPos(AXIS_Z);
    
    print.Data(82)                  = RL_Foot.B_RefPos(AXIS_X);
    print.Data(83)                  = RL_Foot.B_RefPos(AXIS_Y);
    print.Data(84)                  = RL_Foot.B_RefPos(AXIS_Z);
    
    print.Data(85)                  = RR_Foot.B_RefPos(AXIS_X);
    print.Data(86)                  = RR_Foot.B_RefPos(AXIS_Y);
    print.Data(87)                  = RR_Foot.B_RefPos(AXIS_Z);        
    
    print.Data(88)                  = FL_Foot.B_CurrentPos(AXIS_X);
    print.Data(89)                  = FL_Foot.B_CurrentPos(AXIS_Y);
    print.Data(90)                  = FL_Foot.B_CurrentPos(AXIS_Z);
    
    print.Data(91)                  = FR_Foot.B_CurrentPos(AXIS_X);
    print.Data(92)                  = FR_Foot.B_CurrentPos(AXIS_Y);
    print.Data(93)                  = FR_Foot.B_CurrentPos(AXIS_Z);
    
    print.Data(94)                  = RL_Foot.B_CurrentPos(AXIS_X);
    print.Data(95)                  = RL_Foot.B_CurrentPos(AXIS_Y);
    print.Data(96)                  = RL_Foot.B_CurrentPos(AXIS_Z);
    
    print.Data(97)                  = RR_Foot.B_CurrentPos(AXIS_X);
    print.Data(98)                  = RR_Foot.B_CurrentPos(AXIS_Y);
    print.Data(99)                  = RR_Foot.B_CurrentPos(AXIS_Z);
    
    // Foot Vel in Base Frame
    print.Data(100)                 = FL_Foot.B_RefVel(AXIS_X);
    print.Data(101)                 = FL_Foot.B_RefVel(AXIS_Y);
    print.Data(102)                 = FL_Foot.B_RefVel(AXIS_Z);
    
    print.Data(103)                 = FR_Foot.B_RefVel(AXIS_X);
    print.Data(104)                 = FR_Foot.B_RefVel(AXIS_Y);
    print.Data(105)                 = FR_Foot.B_RefVel(AXIS_Z);
    
    print.Data(106)                 = RL_Foot.B_RefVel(AXIS_X);
    print.Data(107)                 = RL_Foot.B_RefVel(AXIS_Y);
    print.Data(108)                 = RL_Foot.B_RefVel(AXIS_Z);
    
    print.Data(109)                 = RR_Foot.B_RefVel(AXIS_X);
    print.Data(110)                 = RR_Foot.B_RefVel(AXIS_Y);
    print.Data(111)                 = RR_Foot.B_RefVel(AXIS_Z);        
    
    print.Data(112)                 = FL_Foot.B_CurrentVel(AXIS_X);
    print.Data(113)                 = FL_Foot.B_CurrentVel(AXIS_Y);
    print.Data(114)                 = FL_Foot.B_CurrentVel(AXIS_Z);
    
    print.Data(115)                 = FR_Foot.B_CurrentVel(AXIS_X);
    print.Data(116)                 = FR_Foot.B_CurrentVel(AXIS_Y);
    print.Data(117)                 = FR_Foot.B_CurrentVel(AXIS_Z);
    
    print.Data(118)                 = RL_Foot.B_CurrentVel(AXIS_X);
    print.Data(119)                 = RL_Foot.B_CurrentVel(AXIS_Y);
    print.Data(120)                 = RL_Foot.B_CurrentVel(AXIS_Z);
    
    print.Data(121)                 = RR_Foot.B_CurrentVel(AXIS_X);
    print.Data(122)                 = RR_Foot.B_CurrentVel(AXIS_Y);
    print.Data(123)                 = RR_Foot.B_CurrentVel(AXIS_Z);

    // Phase
    print.Data(124)                 = walking.phase;

    // Contact State
    print.Data(125)                 = contact.Foot(FL_FootIndex);
    print.Data(126)                 = contact.Foot(FR_FootIndex);
    print.Data(127)                 = contact.Foot(RL_FootIndex);
    print.Data(128)                 = contact.Foot(RR_FootIndex);
    
    // Joint RefPosition
    print.Data(129)                 = joint[FLHR].RefPos;
    print.Data(130)                 = joint[FLHP].RefPos;
    print.Data(131)                 = joint[FLKN].RefPos;
    print.Data(132)                 = joint[FRHR].RefPos;
    print.Data(133)                 = joint[FRHP].RefPos;
    print.Data(134)                 = joint[FRKN].RefPos;
    print.Data(135)                 = joint[RLHR].RefPos;
    print.Data(136)                 = joint[RLHP].RefPos;
    print.Data(137)                 = joint[RLKN].RefPos;
    print.Data(138)                 = joint[RRHR].RefPos;
    print.Data(139)                 = joint[RRHP].RefPos;
    print.Data(140)                 = joint[RRKN].RefPos;
                
    // Joint CurrentPosition
    print.Data(141)                 = joint[FLHR].CurrentPos;
    print.Data(142)                 = joint[FLHP].CurrentPos;
    print.Data(143)                 = joint[FLKN].CurrentPos;
    print.Data(144)                 = joint[FRHR].CurrentPos;
    print.Data(145)                 = joint[FRHP].CurrentPos;
    print.Data(146)                 = joint[FRKN].CurrentPos;
    print.Data(147)                 = joint[RLHR].CurrentPos;
    print.Data(148)                 = joint[RLHP].CurrentPos;
    print.Data(149)                 = joint[RLKN].CurrentPos;
    print.Data(150)                 = joint[RRHR].CurrentPos;
    print.Data(151)                 = joint[RRHP].CurrentPos;
    print.Data(152)                 = joint[RRKN].CurrentPos;

    // Joint RefVelocity
    print.Data(153)                 = joint[FLHR].RefVel;
    print.Data(154)                 = joint[FLHP].RefVel;
    print.Data(155)                 = joint[FLKN].RefVel;
    print.Data(156)                 = joint[FRHR].RefVel;
    print.Data(157)                 = joint[FRHP].RefVel;
    print.Data(158)                 = joint[FRKN].RefVel;
    print.Data(159)                 = joint[RLHR].RefVel;
    print.Data(160)                 = joint[RLHP].RefVel;
    print.Data(161)                 = joint[RLKN].RefVel;
    print.Data(162)                 = joint[RRHR].RefVel;
    print.Data(163)                 = joint[RRHP].RefVel;
    print.Data(164)                 = joint[RRKN].RefVel;
            
    // Joint CurrentVelocity
    print.Data(165)                 = joint[FLHR].CurrentVel;
    print.Data(166)                 = joint[FLHP].CurrentVel;
    print.Data(167)                 = joint[FLKN].CurrentVel;
    print.Data(168)                 = joint[FRHR].CurrentVel;
    print.Data(169)                 = joint[FRHP].CurrentVel;
    print.Data(170)                 = joint[FRKN].CurrentVel;
    print.Data(171)                 = joint[RLHR].CurrentVel;
    print.Data(172)                 = joint[RLHP].CurrentVel;
    print.Data(173)                 = joint[RLKN].CurrentVel;
    print.Data(174)                 = joint[RRHR].CurrentVel;
    print.Data(175)                 = joint[RRHP].CurrentVel;
    print.Data(176)                 = joint[RRKN].CurrentVel;
            
    //Total Ref Torque
    print.Data(177)                 = joint[FLHR].RefTorque + friction.Current[FLHR];
    print.Data(178)                 = joint[FLHP].RefTorque + friction.Current[FLHP];
    print.Data(179)                 = joint[FLKN].RefTorque + friction.Current[FLKN];
    print.Data(180)                 = joint[FRHR].RefTorque + friction.Current[FRHR];
    print.Data(181)                 = joint[FRHP].RefTorque + friction.Current[FRHP];
    print.Data(182)                 = joint[FRKN].RefTorque + friction.Current[FRKN];
    print.Data(183)                 = joint[RLHR].RefTorque + friction.Current[RLHR];
    print.Data(184)                 = joint[RLHP].RefTorque + friction.Current[RLHP];
    print.Data(185)                 = joint[RLKN].RefTorque + friction.Current[RLKN];
    print.Data(186)                 = joint[RRHR].RefTorque + friction.Current[RRHR];
    print.Data(187)                 = joint[RRHP].RefTorque + friction.Current[RRHP];
    print.Data(188)                 = joint[RRKN].RefTorque + friction.Current[RRKN];
               
    //Total Current Torque 
    print.Data(189)                 = joint[FLHR].CurrentTorque;
    print.Data(190)                 = joint[FLHP].CurrentTorque;
    print.Data(191)                 = joint[FLKN].CurrentTorque;
    print.Data(192)                 = joint[FRHR].CurrentTorque;
    print.Data(193)                 = joint[FRHP].CurrentTorque;
    print.Data(194)                 = joint[FRKN].CurrentTorque;
    print.Data(195)                 = joint[RLHR].CurrentTorque;
    print.Data(196)                 = joint[RLHP].CurrentTorque;
    print.Data(197)                 = joint[RLKN].CurrentTorque;
    print.Data(198)                 = joint[RRHR].CurrentTorque;
    print.Data(199)                 = joint[RRHP].CurrentTorque;
    print.Data(200)                 = joint[RRKN].CurrentTorque;

    //Total Current Torque 
    print.Data(201)                 = osqp.torque(6 + FLHR);
    print.Data(202)                 = osqp.torque(6 + FLHP);
    print.Data(203)                 = osqp.torque(6 + FLKN);
    print.Data(204)                 = osqp.torque(6 + FRHR);
    print.Data(205)                 = osqp.torque(6 + FRHP);
    print.Data(206)                 = osqp.torque(6 + FRKN);
    print.Data(207)                 = osqp.torque(6 + RLHR);
    print.Data(208)                 = osqp.torque(6 + RLHP);
    print.Data(209)                 = osqp.torque(6 + RLKN);
    print.Data(210)                 = osqp.torque(6 + RRHR);
    print.Data(211)                 = osqp.torque(6 + RRHP);
    print.Data(212)                 = osqp.torque(6 + RRKN);    
                
    print.Data(213)                 = torque.joint(6 + FLHR);
    print.Data(214)                 = torque.joint(6 + FLHP);
    print.Data(215)                 = torque.joint(6 + FLKN);
    print.Data(216)                 = torque.joint(6 + FRHR);
    print.Data(217)                 = torque.joint(6 + FRHP);
    print.Data(218)                 = torque.joint(6 + FRKN);
    print.Data(219)                 = torque.joint(6 + RLHR);
    print.Data(220)                 = torque.joint(6 + RLHP);
    print.Data(221)                 = torque.joint(6 + RLKN);
    print.Data(222)                 = torque.joint(6 + RRHR);
    print.Data(223)                 = torque.joint(6 + RRHP);
    print.Data(224)                 = torque.joint(6 + RRKN);
            
    print.Data(225)                 = C_term(6 + FLHR);
    print.Data(226)                 = C_term(6 + FLHP);
    print.Data(227)                 = C_term(6 + FLKN);
    print.Data(228)                 = C_term(6 + FRHR);
    print.Data(229)                 = C_term(6 + FRHP);
    print.Data(230)                 = C_term(6 + FRKN);
    print.Data(231)                 = C_term(6 + RLHR);
    print.Data(232)                 = C_term(6 + RLHP);
    print.Data(233)                 = C_term(6 + RLKN);
    print.Data(234)                 = C_term(6 + RRHR);
    print.Data(235)                 = C_term(6 + RRHP);
    print.Data(236)                 = C_term(6 + RRKN);

    print.Data(237)                 = G_term(6 + FLHR);
    print.Data(238)                 = G_term(6 + FLHP);
    print.Data(239)                 = G_term(6 + FLKN);
    print.Data(240)                 = G_term(6 + FRHR);
    print.Data(241)                 = G_term(6 + FRHP);
    print.Data(242)                 = G_term(6 + FRKN);
    print.Data(243)                 = G_term(6 + RLHR);
    print.Data(244)                 = G_term(6 + RLHP);
    print.Data(245)                 = G_term(6 + RLKN);
    print.Data(246)                 = G_term(6 + RRHR);
    print.Data(247)                 = G_term(6 + RRHP);
    print.Data(248)                 = G_term(6 + RRKN);

    print.Data(249)                 = friction.Current[FLHR];
    print.Data(250)                 = friction.Current[FLHP];
    print.Data(251)                 = friction.Current[FLKN];
    print.Data(252)                 = friction.Current[FRHR];
    print.Data(253)                 = friction.Current[FRHP];
    print.Data(254)                 = friction.Current[FRKN];
    print.Data(255)                 = friction.Current[RLHR];
    print.Data(256)                 = friction.Current[RLHP];
    print.Data(257)                 = friction.Current[RLKN];
    print.Data(258)                 = friction.Current[RRHR];
    print.Data(259)                 = friction.Current[RRHP];
    print.Data(260)                 = friction.Current[RRKN];

    
    print.Data(261)                 = M_term_Torque(6 + FLHR);
    print.Data(262)                 = M_term_Torque(6 + FLHP);
    print.Data(263)                 = M_term_Torque(6 + FLKN);
    print.Data(264)                 = M_term_Torque(6 + FRHR);
    print.Data(265)                 = M_term_Torque(6 + FRHP);
    print.Data(266)                 = M_term_Torque(6 + FRKN);
    print.Data(267)                 = M_term_Torque(6 + RLHR);
    print.Data(268)                 = M_term_Torque(6 + RLHP);
    print.Data(269)                 = M_term_Torque(6 + RLKN);
    print.Data(270)                 = M_term_Torque(6 + RRHR);
    print.Data(271)                 = M_term_Torque(6 + RRHP);
    print.Data(272)                 = M_term_Torque(6 + RRKN);
    
    //Time Series
    // print.Data(273)                 = soem.QPThreadTime;
    // print.Data(274)                 = soem.TrajThreadTime;
    // print.Data(275)                 = soem.MotionThreadTime;
    // print.Data(276)                 = soem.IMUThreadTime;
    
    // print.Data(280)                 = soem.StatusWord[FLHR];
    // print.Data(281)                 = soem.StatusWord[FLHP];
    // print.Data(282)                 = soem.StatusWord[FLKN];
    // print.Data(283)                 = soem.StatusWord[FRHR];
    // print.Data(284)                 = soem.StatusWord[FRHP];
    // print.Data(285)                 = soem.StatusWord[FRKN];
    // print.Data(286)                 = soem.StatusWord[RLHR];
    // print.Data(287)                 = soem.StatusWord[RLHP];
    // print.Data(288)                 = soem.StatusWord[RLKN];
    // print.Data(289)                 = soem.StatusWord[RRHR];
    // print.Data(290)                 = soem.StatusWord[RRHP];
    // print.Data(291)                 = soem.StatusWord[RRKN];


    print.Data(273)                 = FL_Foot.GRF_estimated[0];
    print.Data(274)                 = FL_Foot.GRF_estimated[1];
    print.Data(275)                 = FL_Foot.GRF_estimated[2];
    print.Data(276)                 = FL_Foot.distTorque_estimated(0);
    print.Data(277)                 = FL_Foot.distTorque_estimated(1);
    print.Data(278)                 = (FL_Foot.B_CurrentPos(AXIS_Z)*50 + 21.375)*80;
        
    print.Data(292)                 = slope.EstimatedAngle(AXIS_ROLL);
    print.Data(293)                 = slope.EstimatedAngle(AXIS_PITCH);   
    
    print.Data(294)                 = slope.LeastSquareSolution[0];
    print.Data(295)                 = slope.LeastSquareSolution[1];
    print.Data(296)                 = slope.LeastSquareSolution[2];
    
    print.Data(297)                 = walking.step.count;
    
    print.Data(298)                 = base.G_RefAcc(AXIS_X);
    print.Data(299)                 = base.G_RefAcc(AXIS_Y);
    print.Data(300)                 = base.G_RefAcc(AXIS_Z);

    print.Data(301)                 = M_term_hat(6 + FLHR, 6 + FLHR);
    print.Data(302)                 = M_term_hat(6 + FLHP, 6 + FLHP);
    print.Data(303)                 = M_term_hat(6 + FLKN, 6 + FLKN);

    print.Data(304)                 = M_term_hat(6 + FRHR, 6 + FRHR);
    print.Data(305)                 = M_term_hat(6 + FRHP, 6 + FRHP);
    print.Data(306)                 = M_term_hat(6 + FRKN, 6 + FRKN);

    print.Data(307)                 = M_term_hat(6 + RLHR, 6 + RLHR);
    print.Data(308)                 = M_term_hat(6 + RLHP, 6 + RLHP);
    print.Data(309)                 = M_term_hat(6 + RLKN, 6 + RLKN);

    print.Data(310)                 = M_term_hat(6 + RRHR, 6 + RRHR);
    print.Data(311)                 = M_term_hat(6 + RRHP, 6 + RRHP);
    print.Data(312)                 = M_term_hat(6 + RRKN, 6 + RRKN);


    print.Data(313)                 = JointRefAcc(6 + FLHR);
    print.Data(314)                 = JointRefAcc(6 + FLHP);
    print.Data(315)                 = JointRefAcc(6 + FLKN);

    print.Data(316)                 = JointRefAcc(6 + FRHR);
    print.Data(317)                 = JointRefAcc(6 + FRHP);
    print.Data(318)                 = JointRefAcc(6 + FRKN);

    print.Data(319)                 = JointRefAcc(6 + RLHR);
    print.Data(320)                 = JointRefAcc(6 + RLHP);
    print.Data(321)                 = JointRefAcc(6 + RLKN);

    print.Data(322)                 = JointRefAcc(6 + RRHR);
    print.Data(323)                 = JointRefAcc(6 + RRHP);
    print.Data(324)                 = JointRefAcc(6 + RRKN);
    
    print.Data(325)                 = FL_Foot.distTorque_estimated(0);
    print.Data(326)                 = FL_Foot.distTorque_estimated(1);
    print.Data(327)                 = FL_Foot.B_CurrentPos(AXIS_Z)*50;

    print.Data(328)                 = temp_com_ref(0);
    print.Data(329)                 = temp_com_ref(60);
    print.Data(330)                 = temp_com_ref(120);
    print.Data(331)                 = temp_com_ref(180);
    print.Data(332)                 = temp_com_ref(240);

    print.Data(333)                 = temp_legs_ref(0, 1);
    print.Data(334)                 = temp_legs_ref(60, 1);
    print.Data(335)                 = temp_legs_ref(120, 1);
    print.Data(336)                 = temp_legs_ref(180, 1);
    print.Data(337)                 = temp_legs_ref(240, 1);

    print.Data(338)                 = temp_legs_ref(0, 2);
    print.Data(339)                 = temp_legs_ref(60, 2);
    print.Data(340)                 = temp_legs_ref(120, 2);
    print.Data(341)                 = temp_legs_ref(180, 2);
    print.Data(342)                 = temp_legs_ref(240, 2);



    // print.Data()

    // print.Data(338)     = temp_com_ref(0);
    // print.Data(339)     = temp_com_ref(3);
    // print.Data(340)     = temp_com_ref(30);

    // print.Data(341)     = temp_com_vel_ref(0);
    // print.Data(342)     = temp_com_vel_ref(3);
    // print.Data(343)     = temp_com_vel_ref(30); //temp_Foot_to_CoM_ref

    // print.Data(338)     = temp_com_ref(0);
    // print.Data(339)     = temp_com_ref(3);
    // print.Data(340)     = temp_com_ref(30);

    // print.Data(341)     = temp_Foot_to_CoM_ref(0);
    // print.Data(342)     = temp_Foot_to_CoM_ref(3); //1
    // print.Data(343)     = temp_Foot_to_CoM_ref(6); //2
    // print.Data(344)     = temp_Foot_to_CoM_ref(9); // 1
    // print.Data(345)     = temp_Foot_to_CoM_ref(12); // 2
    // print.Data(346)     = temp_Foot_to_CoM_ref(15); //0
    // print.Data(347)     = temp_Foot_to_CoM_ref(18); //1
    // print.Data(348)     = temp_Foot_to_CoM_ref(21); //2
    // print.Data(349)     = temp_Foot_to_CoM_ref(24); //0 osqp.Ref_GRF
    // print.Data(338)     = osqp.Ref_GRF(0);
    // print.Data(339)     = osqp.Ref_GRF(1);
    // print.Data(340)     = osqp.Ref_GRF(2);

    // print.Data(341)     = osqp.Ref_GRF(3);
    // print.Data(342)     = osqp.Ref_GRF(4); //1
    // print.Data(343)     = osqp.Ref_GRF(5); //2
    
    // print.Data(344)     = osqp.Ref_GRF(6); // 1
    // print.Data(345)     = osqp.Ref_GRF(7); // 2
    // print.Data(346)     = osqp.Ref_GRF(8); //0
    
    // print.Data(347)     = osqp.Ref_GRF(9); //1
    // print.Data(348)     = osqp.Ref_GRF(10); //2
    // print.Data(349)     = osqp.Ref_GRF(11); //0 osqp.Ref_GRF osqp.X_mpc 

    print.Data(350)     = osqp.X_mpc(3);  // RL: [58]
    print.Data(351)     = osqp.X_mpc(4);  // 
    print.Data(352)     = osqp.X_mpc(5);  // 

    print.Data(353)     = osqp.X_ref_horizon(3);
    print.Data(354)     = osqp.X_ref_horizon(4); // 
    print.Data(355)     = osqp.X_ref_horizon(5);  //

    print.Data(356)     = osqp.X_ref_horizon(3+13);
    print.Data(357)     = osqp.X_ref_horizon(4+13);
    print.Data(358)     = osqp.X_ref_horizon(5+13);

    print.Data(359)     = base.G_GroundTruthVel(0);
    print.Data(360)     = base.G_GroundTruthVel(1);
    print.Data(361)     = base.G_GroundTruthVel(2);

    print.Data(362)     = base.G_GroundTruthPos(0);
    print.Data(363)     = base.G_GroundTruthPos(1);
    print.Data(364)     = base.G_GroundTruthPos(2);
    


    // print.Data(344)     = FL_Foot.G_RefPos(0+AXIS_Z);  // FL: [52]
    // print.Data(345)     = temp_legs_ref(30+AXIS_Z, 0);
    // print.Data(346)     = temp_legs_ref(60+AXIS_Z, 0);

    // print.Data(347)     = FR_Foot.G_RefPos(0+AXIS_Z);  // FR: [55]
    // print.Data(348)     = temp_legs_ref(30+AXIS_Z, 1);
    // print.Data(349)     = temp_legs_ref(60+AXIS_Z, 1);

    // print.Data(350)     = RL_Foot.G_RefPos(0);  // RL: [58]
    // print.Data(351)     = temp_legs_ref(30, 2);  // 
    // print.Data(352)     = temp_legs_ref(60, 2);  // 

    // print.Data(353)     = RR_Foot.G_RefPos(0);  // FR, X
    // print.Data(354)     = temp_legs_ref(30, 3);  // 
    // print.Data(355)     = temp_legs_ref(60, 3);  // 

    // print.Data(356)     = FL_Foot.G_RefPos(1);  // FL: [52]
    // print.Data(357)     = temp_legs_ref(31, 0);
    // print.Data(358)     = temp_legs_ref(61, 0);

    // print.Data(359)     = FR_Foot.G_RefPos(1);  // FR: [55]
    // print.Data(360)     = temp_legs_ref(31, 1);
    // print.Data(361)     = temp_legs_ref(61, 1);

    // print.Data(362)     = RL_Foot.G_RefPos(1);  // RL: [58]
    // print.Data(363)     = temp_legs_ref(31, 2);  // 
    // print.Data(364)     = temp_legs_ref(61, 2);  // 

    print.Data(365)     = RR_Foot.G_RefPos(1);  // FR, Y
    print.Data(366)     = temp_legs_ref(31, 3);  // 
    print.Data(367)     = temp_legs_ref(61, 3);  // 


    //temp_contact_foots(FL_FootIndex, k)
    print.Data(368)     = temp_contact_foots(FL_FootIndex, 0);
    print.Data(369)     = temp_contact_foots(FL_FootIndex, 1);
    print.Data(370)     = temp_contact_foots(FL_FootIndex, 10);
    print.Data(371)     = temp_contact_foots(FL_FootIndex, 20);
    print.Data(372)     = temp_contact_foots(FL_FootIndex, 30);
    print.Data(373)     = temptime;

    // print.Data(368)     = FL_Foot.G_RefPos(2);  // FL: [52]
    // print.Data(369)     = temp_legs_ref(32, 0);
    // print.Data(370)     = temp_legs_ref(62, 0);

    // print.Data(371)     = FR_Foot.G_RefPos(2);  // FR: [55]
    // print.Data(372)     = temp_legs_ref(32, 1);
    // print.Data(373)     = temp_legs_ref(62, 1);

    // print.Data(374)     = RL_Foot.G_RefPos(2);  // RL: [58]
    // print.Data(375)     = temp_legs_ref(32, 2);  // 
    // print.Data(376)     = temp_legs_ref(62, 2);  // 

    // print.Data(377)     = RR_Foot.G_RefPos(2);  // FR, Z
    // print.Data(378)     = temp_legs_ref(32, 3);  // 
    // print.Data(379)     = temp_legs_ref(62, 3);  // 
    
} 




void CRobot::IMU_Update(){
    imu.DeltaOri_ZYX                = imu.TempOri_ZYX - imu.PreTempOri_ZYX;

    if (imu.DeltaOri_ZYX(AXIS_YAW) > 350*D2R){
        imu.DeltaOri_ZYX(AXIS_YAW)      = 360*D2R - imu.DeltaOri_ZYX(AXIS_YAW); 
    }
    else if (imu.DeltaOri_ZYX(AXIS_YAW) < -350*D2R){
        imu.DeltaOri_ZYX(AXIS_YAW)      = -360*D2R - imu.DeltaOri_ZYX(AXIS_YAW);
    }
    else{
        imu.DeltaOri_ZYX(AXIS_YAW)      = imu.TempOri_ZYX(AXIS_YAW) - imu.PreTempOri_ZYX(AXIS_YAW);
    }

    imu.Ori_EulerZYX                = imu.PreOri_ZYX + imu.DeltaOri_ZYX;
    imu.PreOri_ZYX                  = imu.Ori_EulerZYX;
    imu.PreTempOri_ZYX              = imu.TempOri_ZYX; 

    imu.Ori(AXIS_ROLL)              = imu.Ori_EulerZYX(AXIS_ROLL)   + imu.Offset(AXIS_ROLL);
    imu.Ori(AXIS_PITCH)             = imu.Ori_EulerZYX(AXIS_PITCH)  + imu.Offset(AXIS_PITCH);
    imu.Ori(AXIS_YAW)               = imu.Ori_EulerZYX(AXIS_YAW)    + imu.Offset(AXIS_YAW);
}

void CRobot::IMUNulling(){
    if (Mode == ACTUAL_ROBOT){
        imu.Offset(AXIS_YAW)                      = -imu.Ori_EulerZYX(AXIS_YAW);
    }
    else if(Mode == SIMULATION){
        
    }
}

float CRobot::cosWave(float Amp, float Period, float Time, float InitPos){
    /* cosWave
     * input : Amp, Period, Time, G_InitPos
     * output : Generate CosWave Trajectory
     */
    return (Amp / 2)*(1 - cos(PI / Period * Time)) + InitPos;
}

Vector3f CRobot::cosWave(Vector3f Amp, float Period, float Time, Vector3f InitPos) {
    /* cosWave
     * input : Amp, Period, Time, G_InitPos
     * output : Generate CosWave Trajectory
     */
    Vector3f value;

    value(0) = (Amp(0) / 2)*(1 - cos(PI / Period * Time)) + InitPos(0);
    value(1) = (Amp(1) / 2)*(1 - cos(PI / Period * Time)) + InitPos(1);
    value(2) = (Amp(2) / 2)*(1 - cos(PI / Period * Time)) + InitPos(2);

    return value;
}

float CRobot::differential_cosWave(float Amp, float Period, float Time) {
    /* differential_cosWave
     * input : Amp, Period, Time
     * output : Generate Differential CosWave Trajectory
     */
    return (Amp / 2)*(PI / Period)*(sin(PI / Period * Time));
}

Vector3f CRobot::differential_cosWave(Vector3f Amp, float Period, float Time) {
    /* differential_cosWave
     * input : Amp, Period, Time
     * output : Generate Differential CosWave Trajectory
     */
    Vector3f value;

    value(0) = (Amp(0) / 2)*(PI / Period)*(sin(PI / Period * Time));
    value(1) = (Amp(1) / 2)*(PI / Period)*(sin(PI / Period * Time));
    value(2) = (Amp(2) / 2)*(PI / Period)*(sin(PI / Period * Time));

    return value;
}

MatrixXf CRobot::X_Rot(float q, int size) {
    /* X_Rot
     * input : q, size
     * output : Create Rotation Matrix (X-Axis)
     */
    MatrixXf tmp_m(size, size);

    if(size == 3){
        tmp_m   <<  1,       0,         0, \
                    0,  cos(q),   -sin(q), \
                    0,  sin(q),    cos(q) ;
    }
    else if(size == 4){
        tmp_m   <<  1,        0,        0,   0, \
                    0,   cos(q),  -sin(q),   0, \
                    0,   sin(q),   cos(q),   0, \
                    0,        0,        0,   1 ;
    }
    return tmp_m;
}

MatrixXf CRobot::Y_Rot(float q, int size) {
    /* Y_Rot
     * input : q, size
     * output : Create Rotation Matrix (Y-Axis)
     */
    MatrixXf tmp_m(size, size);

    if(size == 3){
        tmp_m <<    cos(q),     0,  sin(q), \
                         0,     1,       0, \
                   -sin(q),     0,  cos(q) ;
    }
    else if(size == 4){
        tmp_m <<    cos(q),      0,   sin(q),  0, \
                         0,      1,        0,  0, \
                   -sin(q),      0,   cos(q),  0, \
                         0,      0,        0,  1 ;
    }
    return tmp_m;
}

MatrixXf CRobot::Z_Rot(float q, int size) {
    /* Z_Rot
     * input : q, size
     * output : Create Rotation Matrix (Z-Axis)
     */
    MatrixXf tmp_m(size, size);

    if(size == 3){
        tmp_m <<    cos(q),   -sin(q),   0, \
                    sin(q),    cos(q),   0, \
                         0,         0,   1;
    }
    else if(size == 4){
        tmp_m <<    cos(q),   -sin(q),   0,   0, \
                    sin(q),    cos(q),   0,   0, \
                         0,         0,   1,   0, \
                         0,         0,   0,   1 ;
    }
    return tmp_m;
}

Matrix3f CRobot::EulerZYX2RotMat(Vector3f RotationAngle) {
    
    Matrix3f tmp_m;
    
    tmp_m = Z_Rot(RotationAngle(AXIS_YAW), 3)*Y_Rot(RotationAngle(AXIS_PITCH), 3)*X_Rot(RotationAngle(AXIS_ROLL), 3);
    return tmp_m;
}

void CRobot::setRotationZYX(Vector3f ori, float** out_T)
{
    // set Rotation Matrix Using (1.rotation ZYX Euler Angles)
    // ori is in order of roll-pitch-yaw

    float Cx = cos(ori(0));
    float Cy = cos(ori(1));
    float Cz = cos(ori(2));
    float Sx = sin(ori(0));
    float Sy = sin(ori(1));
    float Sz = sin(ori(2));

    out_T[1][1] = Cz * Cy;
    out_T[1][2] = Cz * Sy * Sx - Sz * Cx;
    out_T[1][3] = Cz * Sy * Cx + Sz * Sx;
    out_T[2][1] = Sz * Cy;
    out_T[2][2] = Sz * Sy * Sx + Cz * Cx;
    out_T[2][3] = Sz * Sy * Cx - Cz * Sx;
    out_T[3][1] = -Sy;
    out_T[3][2] = Cy * Sx;
    out_T[3][3] = Cy * Cx;
}

Vector3f CRobot::RotMat2EulerZYX(Matrix3f RotMat) {
    /* RotationMatrix -> EulerAngleZYX
     * input : RotationMatrix
     * output : EulerAngleZYX
     */
    Vector3f RotationAngle;
    //
    RotationAngle(AXIS_ROLL)    = atan2( RotMat(2,1), RotMat(2,2));    
    RotationAngle(AXIS_PITCH)   = atan2(-RotMat(2,0), sqrt(RotMat(2,1)*RotMat(2,1)+RotMat(2,2)*RotMat(2,2)));
    RotationAngle(AXIS_YAW)     = atan2( RotMat(1,0), RotMat(0,0));
    
    return RotationAngle;
}

void CRobot::setMappingE_ZYX(Vector3f ori, float** out_T)
{
    // mapping E for time derivatives of ZYX Euler Angles
    // ori is in order of roll-pitch-yaw
    // out_T are in order of roll-pitch-yaw
    
    //* E_R_eulerZYX = [E(Z_AXIS) E(Y_AXIS) E(X_AXIS)]      ->      [E(X_AXIS) E(Y_AXIS) E(Z_AXIS)]
    //               = [    0        -Sz     Cy * Cz                [ Cy * Cz     -Sz        0
    //                      0         Cz     Cy * Sz                  Cy * Sz      Cz        0
    //                      1         0        -Sy   ]                  -Sy        0         1    ]

    float Cx = cos(ori(0));
    float Cy = cos(ori(1));
    float Cz = cos(ori(2));
    float Sx = sin(ori(0));
    float Sy = sin(ori(1));
    float Sz = sin(ori(2));

    
    out_T[1][1] = Cy * Cz;
    out_T[1][2] = -Sz;
    out_T[1][3] = 0;
    out_T[2][1] = Cy * Sz;
    out_T[2][2] = Cz;
    out_T[2][3] = 0;
    out_T[3][1] = -Sy;
    out_T[3][2] = 0;
    out_T[3][3] = 1;
}

void CRobot::invMappingE_ZYX(Vector3f ori, float** out_T)
{
    // out_T is the inverse matrix for mapping E to time derivatives of ZYX Euler Angles
    // ori is in order of roll-pitch-yaw
    // out_T are in order of roll-pitch-yaw

    //* E_R_eulerZYX_inv    = [   E(X_AXIS)       E(Y_AXIS)       E(Z_AXIS)   ]      
    //                      = [    Cz / Cy         Sz / Cy            0
    //                               -Sz             Cz               0              
    //                          Cz * Sy / Cy     Sy * Sz / Cy         1       ]      
    float Cx = cos(ori(0));
    float Cy = cos(ori(1));
    float Cz = cos(ori(2));
    float Sx = sin(ori(0));
    float Sy = sin(ori(1));
    float Sz = sin(ori(2));

    out_T[1][1] = Cz / Cy;
    out_T[1][2] = Sz / Cy;
    out_T[1][3] = 0;
    out_T[2][1] = -Sz;
    out_T[2][2] = Cz;
    out_T[2][3] = 0;
    out_T[3][1] = Cz * Sy / Cy;
    out_T[3][2] = Sz * Sy / Cy;
    out_T[3][3] = 1;
}

void CRobot::set_skew_symmetic(Vector3f vec, float** out_T)
{
    out_T[1][1] = 0;
    out_T[1][2] = -vec(2);
    out_T[1][3] = vec(1);

    out_T[2][1] = vec(2);
    out_T[2][2] = 0;
    out_T[2][3] = -vec(0);

    out_T[3][1] = -vec(1);
    out_T[3][2] = vec(0);
    out_T[3][3] = 0;
}

Vector3f CRobot::local2Global(Vector3f Vector, Matrix3f RotMat){

    Vector3f outputVector, inputVector;
    
    inputVector(AXIS_X)     = Vector(AXIS_X);
    inputVector(AXIS_Y)     = Vector(AXIS_Y);
    inputVector(AXIS_Z)     = Vector(AXIS_Z);

    outputVector            = RotMat*inputVector;

    return outputVector;
}

Vector3f CRobot::global2Local(Vector3f Vector, Matrix3f RotMat){

    Vector3f outputVector, inputVector;
    
    inputVector(AXIS_X)     = Vector(AXIS_X);
    inputVector(AXIS_Y)     = Vector(AXIS_Y);
    inputVector(AXIS_Z)     = Vector(AXIS_Z);

    outputVector            = RotMat.transpose()*inputVector;

    return outputVector;
}

void CRobot::set_5thPolyNomialMatrix(float final_time, float** out_T)
{
    static float InitTime  = 0.0;
    
    for (int index = 1; index <= 6; index++) {
        out_T[1][index] = pow(InitTime , index - 1);
        out_T[2][index] = pow(final_time, index - 1);
    }
    
    for (int index = 1; index <= 6; index++) {
        if (index == 1){
            out_T[3][index] = 0.0;
            out_T[4][index] = 0.0;
        }
        else{
            out_T[3][index] = (index - 1)*pow(InitTime , index - 2);
            out_T[4][index] = (index - 1)*pow(final_time, index - 2);
        }
    }
    
    for (int index = 1; index <= 6; index++) {
        if (index == 1){
            out_T[5][index] = 0.0;
            out_T[6][index] = 0.0;
        }
        else if (index == 2){
            out_T[5][index] = 0.0;
            out_T[6][index] = 0.0;
        }
        else{
            out_T[5][index] = (index - 2)*(index - 1)*pow(InitTime , index - 3);
            out_T[6][index] = (index - 2)*(index - 1)*pow(final_time, index - 3);
        }
    }
}

void CRobot::coefficient_5thPolyNomial(Vector3f pre_value, Vector3f final_value, float final_time, float *output) {
    /* coefficient_5thPolyNomial
     * input : pre_value, final_value, final_time, *output
     * output : Create 5thPolyNomial Coefficient
     */
    Vector6f Coeff_InputState   = Vector6f::Zero(6);
    Matrix6f Coeff_Matrix       = Matrix6f::Zero(6, 6);
    Vector6f Coeff_State        = Vector6f::Zero(6);

    static float InitTime, FinalTime;

    Coeff_InputState    << pre_value[0], final_value[0], pre_value[1], final_value[1], pre_value[2], final_value[2];

    InitTime = 0;
    FinalTime = final_time;

    Coeff_Matrix        <<  1,       InitTime,       pow(InitTime, 2),       pow(InitTime, 3),        pow(InitTime, 4),        pow(InitTime, 5),
                            1,      FinalTime,      pow(FinalTime, 2),      pow(FinalTime, 3),       pow(FinalTime, 4),       pow(FinalTime, 5),
                            0,              1,     2*pow(InitTime, 1),     3*pow(InitTime, 2),      4*pow(InitTime, 3),      5*pow(InitTime, 4),
                            0,              1,    2*pow(FinalTime, 1),    3*pow(FinalTime, 2),     4*pow(FinalTime, 3),     5*pow(FinalTime, 4),
                            0,              0,                      2,     6*pow(InitTime, 1),     12*pow(InitTime, 2),     20*pow(InitTime, 3),
                            0,              0,                      2,    6*pow(FinalTime, 1),    12*pow(FinalTime, 2),    20*pow(FinalTime, 3);

    Coeff_State = Coeff_Matrix.inverse() * Coeff_InputState;    
            
    output[0] = Coeff_State(0, 0);
    output[1] = Coeff_State(1, 0);
    output[2] = Coeff_State(2, 0);
    output[3] = Coeff_State(3, 0);
    output[4] = Coeff_State(4, 0);
    output[5] = Coeff_State(5, 0);
}


float CRobot::Function_5thPolyNomial(Vector6f coeff, float time) {
    /* Function_5thPolyNomial
     * input : coeff[], time
     * output : Generate 5thPolyNomial Trajectory
     */
    static float value = 0;
    value = coeff(5) * pow(time, 5) + coeff(4) * pow(time, 4) + coeff(3) * pow(time, 3) + coeff(2) * pow(time, 2) + coeff(1) * pow(time, 1) + coeff(0);

    return value;
}

float CRobot::Function_5thPolyNomial_dot(Vector6f coeff, float time) {
    /* Function_5thPolyNomial_dot
     * input : coeff[], time
     * output : Generate 5thPolyNomial_dot Trajectory
     */
    static float value = 0;
    value = 5 * coeff(5) * pow(time, 4) + 4 * coeff(4) * pow(time, 3) + 3 * coeff(3) * pow(time, 2) + 2 * coeff(2) * pow(time, 1) + 1 * coeff(1);

    return value;
}

float CRobot::Function_5thPolyNomial_2dot(Vector6f coeff, float time) {
    /* Function_5thPolyNomial_dot
     * input : coeff[], time
     * output : Generate 5thPolyNomial_dot Trajectory
     */
    static float value = 0;
    value = 20 * coeff(5) * pow(time, 3) + 12 * coeff(4) * pow(time, 2) + 6 * coeff(3) * pow(time, 1) + 2 * coeff(2);

    return value;
}




VectorXf CRobot::Set_8thBezierControlPoint(Vector3f StartPoint, Vector3f EndPoint, int Axis, Matrix3f Rotation) {
    /* Set_8thBezierControlPoint
     * input : PreValue, FinalValue, Period, Time
     * output : Set 8thBezierCurve ControlPoint
     */
    const float offset       = 0.2;
    VectorXf ControlPoint     = VectorXf::Zero(9);
    Vector3f tmp_m;
    
    if(Axis == 0){
        ControlPoint(0)           = StartPoint(0);
        ControlPoint(1)           = StartPoint(0);
        ControlPoint(2)           = StartPoint(0);

        tmp_m << 0 - 2* offset, 0, 0;
        ControlPoint(3)           = StartPoint(0) + local2Global(tmp_m, Rotation)(0);
        ControlPoint(4)           = (StartPoint(0) + EndPoint(0))/2.0;
        tmp_m << offset, 0, 0;
        ControlPoint(5)           = EndPoint(0)   + local2Global(tmp_m, Rotation)(0);
        
        ControlPoint(6)           = EndPoint(0);
        ControlPoint(7)           = EndPoint(0);
        ControlPoint(8)           = EndPoint(0);
    }
    else if(Axis == 1){
        ControlPoint(0)           = StartPoint(1);
        ControlPoint(1)           = StartPoint(1);
        ControlPoint(2)           = StartPoint(1);
        
        ControlPoint(3)           = StartPoint(1);
        ControlPoint(4)           = (StartPoint(1) + EndPoint(1))/2.0;
        ControlPoint(5)           = EndPoint(1);
        
        ControlPoint(6)           = EndPoint(1);
        ControlPoint(7)           = EndPoint(1);
        ControlPoint(8)           = EndPoint(1);
        
    }
    else if(Axis == 2){
        ControlPoint(0)           = StartPoint(2);
        ControlPoint(1)           = StartPoint(2);
        ControlPoint(2)           = StartPoint(2);
        
        ControlPoint(3)           = (StartPoint(2) + EndPoint(2))/2.0;
        ControlPoint(4)           = (StartPoint(2) + EndPoint(2))/2.0;
        ControlPoint(5)           = (StartPoint(2) + EndPoint(2))/2.0;
        
        ControlPoint(6)           = EndPoint(2);
        ControlPoint(7)           = EndPoint(2);
        ControlPoint(8)           = EndPoint(2);
    }
    
    return ControlPoint;
}

float CRobot::Function_8thBezierCurve(VectorXf ControlPoint, float Period, float Time) {
    /* Function_8thBezierCurve
     * input : pre_value, final_value, time
     * output : Generate 8thBezierCurve Trajectory
     */
    float value                = 0.0;
    VectorXf BezierValue        = VectorXf::Zero(9);
    
    BezierValue(0)              =           pow(1 - (Time/Period), 8);
    BezierValue(1)              =     8 *   pow(1 - (Time/Period), 7)   *       (Time/Period);
    BezierValue(2)              =    28 *   pow(1 - (Time/Period), 6)   *   pow((Time/Period), 2);
    BezierValue(3)              =    56 *   pow(1 - (Time/Period), 5)   *   pow((Time/Period), 3);
    BezierValue(4)              =    70 *   pow(1 - (Time/Period), 4)   *   pow((Time/Period), 4);
    BezierValue(5)              =    56 *   pow(1 - (Time/Period), 3)   *   pow((Time/Period), 5);
    BezierValue(6)              =    28 *   pow(1 - (Time/Period), 2)   *   pow((Time/Period), 6);
    BezierValue(7)              =     8 *      (1 - (Time/Period))      *   pow((Time/Period), 7);
    BezierValue(8)              =                                           pow((Time/Period), 8);
        
    for (int i = 0; i < 9; ++i){
        value =  value + ControlPoint[i]*BezierValue[i];
    }
    return value;
}

float CRobot::Function_8thBezierCurve_dot(VectorXf ControlPoint, float Period, float Time) {
    /* Function_8thBezierCurve_dot
     * input : pre_value, final_value, time
     * output : Generate 8thBezierCurve Trajectory
     */
    float value                = 0.0;
    
    VectorXf BezierValue        = VectorXf::Zero(9);
    
    BezierValue(0)              =     8 * pow(1 - (Time/Period), 7) * (-1.0/Period);
    BezierValue(1)              =    56 * pow(1 - (Time/Period), 6) * (-1.0/Period) *     (Time/Period)      +   8 * pow(1 - (Time/Period), 7) *                         (1.0/Period);
    BezierValue(2)              =   168 * pow(1 - (Time/Period), 5) * (-1.0/Period) * pow((Time/Period), 2)  +  56 * pow(1 - (Time/Period), 6) *         (Time/Period) * (1.0/Period);
    BezierValue(3)              =   280 * pow(1 - (Time/Period), 4) * (-1.0/Period) * pow((Time/Period), 3)  + 168 * pow(1 - (Time/Period), 5) * pow((Time/Period), 2) * (1.0/Period); 
    BezierValue(4)              =   280 * pow(1 - (Time/Period), 3) * (-1.0/Period) * pow((Time/Period), 4)  + 280 * pow(1 - (Time/Period), 4) * pow((Time/Period), 3) * (1.0/Period);  
    BezierValue(5)              =   168 * pow(1 - (Time/Period), 2) * (-1.0/Period) * pow((Time/Period), 5)  + 280 * pow(1 - (Time/Period), 3) * pow((Time/Period), 4) * (1.0/Period);  
    BezierValue(6)              =    56 *       (1 - (Time/Period)) * (-1.0/Period) * pow((Time/Period), 6)  + 168 * pow(1 - (Time/Period), 2) * pow((Time/Period), 5) * (1.0/Period);  
    BezierValue(7)              =     8                             * (-1.0/Period) * pow((Time/Period), 7)  +  56 *    (1 - (Time/Period))    * pow((Time/Period), 6) * (1.0/Period);  
    BezierValue(8)              =     8 *                                             pow((Time/Period), 7)                                                            * (1.0/Period);
    
    for (int i = 0; i < 9; ++i){
        value =  value + ControlPoint[i]*BezierValue[i];
    }
    
    return value;
}

void CRobot::MomentumObserver(double f_cutoff)
{
    // RobotJoint: JOINT 포인터 기입. joint 로 채워넣으면 모든 관절의 정보가 들어갈 것
    // inputTorque: 모든 관절에 입력되는 토크 기입
    // LegNumber: FL, FR, RL, RR
    /*
    systemID.RobotThighMass             =   10.417;     //3.32;
    systemID.RobotCalfMass              =   5.500610;   //0.57335;
    systemID.RobotTipMass               =   0.3;        //0.1646;
    lowerbody.HP_TO_KN_LENGTH_Z = 0.5;
    lowerbody.KN_TO_EP_LENGTH_Z = 0.51086 + 0.045;
    joint[nJoint].CurrentPos
    joint[nJoint].torque = CTC_Torque(6 + nJoint
    */
    double m1 = BodyKine.m_FL_thigh_link;
    double m2 = BodyKine.m_FL_calf_link;
    
    
    double l1 = lowerbody.HP_TO_KN_LENGTH(AXIS_Z);
    double l2 = lowerbody.KN_TO_EP_LENGTH(AXIS_Z);

    double dt = tasktime;      // system 데이터 업데이트되는 주기로 바꿔야함
    
    double lambda = 2*M_PI*f_cutoff;
    double alpha = (1/lambda)/(1/lambda + dt);
    
    Vector2d EstimatedTorque;
    
    MatrixXd Mass_term(2,2);
    MatrixXd Mass_term_dot(2,2);
    MatrixXd Coriolis_term(2,2);
    MatrixXd Coriolis_term_transpose(2,2);
    
//    double q1 = -joint[1].CurrentPos - M_PI;
//    double q2 = -joint[2].CurrentPos;
//    double q1_dot = -joint[1].CurrentVel;
//    double q2_dot = -joint[2].CurrentVel;
//    double q1_torque = -joint[1].RefTorque;
//    double q2_torque = -joint[2].RefTorque;
    
    
    // double q1           = joint[1].CurrentPos;
    double q1           = joint[1].CurrentPos - M_PI;
    double q2           = joint[2].CurrentPos;
    double q1_dot       = joint[1].CurrentVel;
    double q2_dot       = joint[2].CurrentVel;
    double q1_torque    = joint[1].RefTorque;
    double q2_torque    = joint[2].RefTorque;
    
    Vector2d q(q1, q2);
    Vector2d q_dot(q1_dot, q2_dot);
    Vector2d tau_input(q1_torque, q2_torque);
    Vector2d Gravity_term;
    // Matrices
    Mass_term << 1/3*m1*l1*l1 + m2*l1*l1 + 1/3*m2*l2*l2 + m2*l1*l2*cos(q2),
                 1/3*m2*l2*l2 + 1/2*m2*l1*l2*cos(q2),
                 1/3*m2*l2*l2 + 1/2*m2*l1*l2*cos(q2),
                 1/3*m2*l2*l2;
    Mass_term_dot << -m2*l1*l2*sin(q2)*q2_dot,
                     -1/2*m2*l1*l2*sin(q2)*q2_dot,
                     -1/2*m2*l1*l2*sin(q2)*q2_dot,
                     0;
    
    Gravity_term << (1/2*m1+m2)*9.81*l1*sin(q1) + 1/2*m2*9.81*l2*sin(q1+q2),
                    1/2*m2*9.81*l2*sin(q1+q2);

    Coriolis_term << -0.5*m2*l1*l2*sin(q2)*q2_dot,
                    -1/2*m2*l1*l2*sin(q2)*(q1_dot + q2_dot),
                    1/2*m2*l1*l2*sin(q2)*q1_dot,
                    0;
    Coriolis_term_transpose = Coriolis_term.transpose();
    // Algorithm
    //       momentum_now = Mass_term(:,:,i) * d1_q(:,i);
    //       temp_now = lambda*momentum_now + Torque_input(:, i) + Christoffel_T(:,:,i)*d1_q(:,i)  - Gravity_term(:,i);
    //       temp_filtered = (1-alpha)*temp_now + alpha*temp_filtered;
    //       Torque_dist_est_new(:,i) = lambda*momentum_now - temp_filtered;
    FL_Foot.temp_now = lambda*(Mass_term*q_dot) + tau_input + Coriolis_term_transpose*q_dot - Gravity_term;
    FL_Foot.temp_filtered = (1-alpha) * FL_Foot.temp_now + alpha*FL_Foot.temp_filtered;
    FL_Foot.distTorque_estimated = lambda*(Mass_term*q_dot) - FL_Foot.temp_filtered;

    FL_Foot.Jacobian_2d << -l1*cos(q1)-l2*cos(q1+q2), -l2*cos(q1+q2), l1*sin(q1)+l2*sin(q1+q2), l2*sin(q1+q2);
    FL_Foot.Jacobian_2d_T = (FL_Foot.Jacobian_2d).transpose();
    FL_Foot.Jacobian_2d_T_inv = (FL_Foot.Jacobian_2d_T).inverse();
    FL_Foot.Jacobian_2d_T_inv_use << FL_Foot.Jacobian_2d_T_inv, 0, 0;  
    
    
    
    
    FL_Foot.GRF_estimated = -FL_Foot.Jacobian_2d_T_inv_use * FL_Foot.distTorque_estimated;

    // std::cout << FL_Foot.GRF_estimated << std::endl;

//    std::cout << "estimated torque[q1; q2]: \n " << FL_Foot.distTorquePr_estimated << std::endl;
    //return Vector2d::Zero();
}







VectorXf CRobot::MPC_controller(unsigned int horizon_length, float samplingtime, unsigned int total_legs = 4)
{
    //** start of 입력으로 받은 변수들 또는 전역 변수들을 지역 변수로 새로 정의
    unsigned int n = total_legs;
    unsigned int l = horizon_length;
    float dt = samplingtime;

    if(l > 10){
        l = 10; // 일단 미래의 갯수는 10개 이하라고 가정하자.
    }
    
    MatrixXf A_c = MatrixXf::Zero(13, 13);  // Define A_c and B_c (in continuous domain)
    MatrixXf B_c = MatrixXf::Zero(13*l, 3*n);


    // A_hat 과 B_hat
    MatrixXf A_hat = MatrixXf::Zero(13, 13);
    MatrixXf B_hat = MatrixXf::Zero(13*l, 3*n);

    MatrixXf w_I_g = MatrixXf::Zero(3, 3);
    MatrixXf w_I_g_transpose = MatrixXf::Zero(3, 3);        // 글로벌 좌표계에서 바라본 몸체의 관성 모멘트와 그의 전치

    float psi = imu.Ori(AXIS_YAW);          // IMU 센서에서 계산되어 나온 오일러각 중 Yaw 의 각도

    Vector3f r_fl = FL_Foot.G_CoM_to_Foot_CurrentPos;   // 글로벌 좌표계에서 바라본 몸체로부터 (왼쪽 앞)발 끝까지의 위치 벡터
    Vector3f r_fr = FR_Foot.G_CoM_to_Foot_CurrentPos;
    Vector3f r_rl = RL_Foot.G_CoM_to_Foot_CurrentPos;
    Vector3f r_rr = RR_Foot.G_CoM_to_Foot_CurrentPos;

    //** 이제 몸체의 글로벌 미래 위치벡터(temp_com_ref) 와 다리의 글로벌 미래 위치벡터들(temp_legs_ref) 를 알고 있으므로,
    //** 이들을 이용하여 글로벌 좌표계에서 몸체에서 바라본 다리의 위치벡터를 구할 수 있다.
    //** 위의 temp 들을 전역으로 만들어둔 게 맞는지 아직 의문이다.
    VectorXf r_fl_vector = VectorXf::Zero(3*l);
    VectorXf r_fr_vector = VectorXf::Zero(3*l);
    VectorXf r_rl_vector = VectorXf::Zero(3*l);
    VectorXf r_rr_vector = VectorXf::Zero(3*l);


    for(size_t i = 0; i < l; ++i){        
        // 추후 여기에서, 패턴 제네레이터(트롯 함수 등) 에서 빠른 속도로 계산된 미래 궤적을, mpc 제어기의 속도에 맞게 띄엄띄엄 샘플링해서 저장하면 된다.
        // temp_com2FL_ref 의 경우, trot 함수에서 계산된 글로벌 좌표계에서의 몸체로부터 다리까지의 위치 벡터이다.
        r_fl_vector.block<3, 1>(3*i, 0) = temp_com2FL_ref.block<3, 1>(3*i, 0);  
        r_fr_vector.block<3, 1>(3*i, 0) = temp_com2FR_ref.block<3, 1>(3*i, 0);  
        r_rl_vector.block<3, 1>(3*i, 0) = temp_com2RL_ref.block<3, 1>(3*i, 0);  
        r_rr_vector.block<3, 1>(3*i, 0) = temp_com2RR_ref.block<3, 1>(3*i, 0);  
    }

    float mass_total = BodyKine.m_total;            // 로봇의 전체 질량

    VectorXf vector_temp = VectorXf::Zero(n);
    for(size_t i = 0; i < n; ++i){
        vector_temp(i) = i;
    }
    // std::cout << "MPC_controller: \n" << vector_temp << std::endl;
    //** end of 입력으로 받은 변수들 또는 전역 변수들을 지역 변수로 새로 정의


    //** 1. Continuous 에서의 A_c 와 B_c 행렬 계산하기
    // A_c
    A_c.block<3, 3>(0, 6) = rot_z(psi).transpose();
    A_c.block<3, 3>(3, 9) = MatrixXf::Identity(3, 3);
    A_c(11, 12) = -1;


    // B_c
    // 1. Calculate w_I_g and w_I_g^T   
    //w_I_g = BodyKine.B_I_g;  
    w_I_g = rot_z(psi).transpose()*BodyKine.B_I_g*rot_z(psi);    // MIT 논문과 Hutter 교재에서의 회전행렬이 다르기 때문에 전치행렬의 순서가 다르다.
    w_I_g_transpose = w_I_g.transpose();

    // std::cout << "psi: " << psi << std::endl;
    // std::cout << "w_I_g_transpose: \n" << w_I_g_transpose << std::endl; 

    // 2. Calculate B_c
    // B_c.block<3, 3>(6, 0) = w_I_g_transpose * skew_matrix(r_fl);
    // B_c.block<3, 3>(6, 3) = w_I_g_transpose * skew_matrix(r_fr);
    // B_c.block<3, 3>(6, 6) = w_I_g_transpose * skew_matrix(r_rl);
    // B_c.block<3, 3>(6, 9) = w_I_g_transpose * skew_matrix(r_rr);
    
    // B_c.block<3, 3>(9, 0) = (1./mass_total)*MatrixXf::Identity(3, 3);
    // B_c.block<3, 3>(9, 3) = (1./mass_total)*MatrixXf::Identity(3, 3);
    // B_c.block<3, 3>(9, 6) = (1./mass_total)*MatrixXf::Identity(3, 3);
    // B_c.block<3, 3>(9, 9) = (1./mass_total)*MatrixXf::Identity(3, 3);
    
    // B_c 를 계산하는 과정에서부터, B_c 매트릭스의 크기에 horizontal length 를 적용해서 크게 계산하는 게 맞을 것 같다
    for(size_t i = 0; i < l; ++i){
        // B_c 는 13*(3*4) 의 행렬이 l 개의 행으로 구성된 (13*l) * (3*4) 행렬.
        B_c.block<3, 3>(13*i + 6, 0) = w_I_g_transpose * skew_matrix(r_fl_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 3) = w_I_g_transpose * skew_matrix(r_fr_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 6) = w_I_g_transpose * skew_matrix(r_rl_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 9) = w_I_g_transpose * skew_matrix(r_rr_vector.block<3, 1>(3*i, 0));

        B_c.block<3, 3>(13*i + 6, 0) = skew_matrix(r_fl_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 3) = skew_matrix(r_fr_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 6) = skew_matrix(r_rl_vector.block<3, 1>(3*i, 0));
        B_c.block<3, 3>(13*i + 6, 9) = skew_matrix(r_rr_vector.block<3, 1>(3*i, 0));


        B_c.block<3, 3>(13*i + 9, 0) = (1./mass_total)*MatrixXf::Identity(3, 3);
        B_c.block<3, 3>(13*i + 9, 3) = (1./mass_total)*MatrixXf::Identity(3, 3);
        B_c.block<3, 3>(13*i + 9, 6) = (1./mass_total)*MatrixXf::Identity(3, 3);
        B_c.block<3, 3>(13*i + 9, 9) = (1./mass_total)*MatrixXf::Identity(3, 3);

    }
    
    
    
    //** end of 1. Continuous 에서의 A_c 와 B_c 행렬 계산하기

    //** 2. Discrete 로 변환하기 (A_hat, B_hat)
    A_hat = A_c * dt + MatrixXf::Identity(13, 13);
    
    // 문제는 B_hat. B_hat 은 B_hat[k], B_hat[k+1], B_hat[k+2], ... , 의 구성으로 돼어있으며,
    // 내부에 있는 몸체로부터 다리까지의 위치벡터가 시간에 대한 미래값으로 들어가야 함.
    // 그렇기 때문에 패턴 제네레이터에 대한 내용이 필요함. (B_c 에서부터 하는 것도 맞다고 생각함) -> B_c 의 사이즈를 k=0 부터 k=l-1 까지 행벡터로 포함하도록 확장하였음 (B_hat 과 동일)
    B_hat = B_c * dt;
    
    // std::cout << "B_hat: \n" << B_hat << std::endl;

    //BodyKine.m_total
    //** end of Discrete 로 변환하기

    //** 3. A_qp 와 B_qp 만들기
    // A_qp 는 A 와 같은 차원의 행렬을 원소로 가지는 열벡터이며, 열의 갯수는 horizontal length l 이다. [A_hat, A_hat^2, A_hat^3, ...].transpose
    // B_qp 는 B_hat[k]= 13*(3*4) 사이즈의 행렬을 원소로 가지며, l*l 이다. 따라서 B_qp 행렬의 크기는 (13*l) * (3*4*l) 이다.
    MatrixXf A_qp = MatrixXf::Zero(13*l, 13);
    MatrixXf B_qp = MatrixXf::Zero(13*l, 3*n*l);

    // Eigen lib 에서 행렬의 거듭제곱을 이용하기 위해서, MatrixPower 자료형이 필요 -> 사용 불가능...
    //MatrixPower<Matrix4f> A_hat_pow(A_hat);
    

    // A_qp 계산 알고리즘
    for(size_t i = 0; i < l; ++i){
        if(i == 0){
            A_qp.block<13, 13>(13*i, 0) = A_hat;
        }
        else{
            A_qp.block<13, 13>(13*i, 0) = A_hat * A_qp.block<13, 13>(13*(i-1), 0);
        }
    }
    // std::cout << "A_qp: \n" << A_qp << std::endl;

    // B_qp 계산 알고리즘
    for(size_t row = 0; row < l; ++row){      // 각 row번째 행에 대해서
        for(size_t col = 0; col < l; ++col){  // 각 col번째 열에 대해서
            if(col <= row){
                B_qp.block<13, 12>(13*row, 12*col) = B_hat.block<13, 12>(13*col, 0);
            }

            if(row > col){
                B_qp.block<13, 12>(13*row, 12*col) = A_qp.block<13, 13>(13*(row-col-1), 0) * B_qp.block<13, 12>(13*row, 12*col);
            }
        }
    }

    //std::cout << "B_qp: \n" << B_qp << std::endl;

    //** End of A_qp 와 B_qp 만들기



    //** 4. A_qp 와 B_qp, weighting matrix 들을 이용하여 Quadratic Programming Solve (by using OSQP)
    // 4-1. desired Ref RPY






    //** End of OSQP
    return vector_temp;
}

Eigen::MatrixXf rot_z(float angle)
{
    Eigen::MatrixXf tmp_m(3, 3);
    //tmp_m << cos(angle), -sin(angle), 0,
    tmp_m << cos(angle), -sin(angle), 0, \
             sin(angle), cos(angle), 0, \
             0, 0, 1;

    return tmp_m;
}

Eigen::MatrixXf rot_y(float angle)
{
    Eigen::MatrixXf tmp_m(3, 3);
    //tmp_m << cos(angle), -sin(angle), 0,
    tmp_m << cos(angle), 0, sin(angle), \
             0, 1, 0,\
             -sin(angle), 0, cos(angle);


    return tmp_m;
}

Eigen::MatrixXf rot_x(float angle)
{
    Eigen::MatrixXf tmp_m(3, 3);
    //tmp_m << cos(angle), -sin(angle), 0,
    tmp_m << 1, 0, 0,\
            0, cos(angle), -sin(angle),\
            0, sin(angle), cos(angle);


    return tmp_m;
}

Matrix3f skew_matrix(Vector3f vec)
{
    Matrix3f temp;
    temp << 0, -vec(2), vec(1),\
            vec(2), 0, -vec(0),\
            -vec(1), vec(0), 0;

    return temp;
}



MatrixXf CRobot::generateFootTraj_Trot_modified(float com_time, float motion_time, int Startfoot, int Startfoot_prev, int k) {
        
    //** FL, RR 의 경우 궤적 생성기를 맨 처음 시작할 때, 튀는 문제가 있다. 오프셋 계산이 불가능하기 때문인데,
    //** 이를 해결해주기 위해 함수 내 지역 변수로 플래그를 하나 만들었다.
    bool flag_StartFootChange = false;
    
    float temp_FL_offset_init = 0.;
    float temp_RR_offset_init = 0.;


    VectorXf temp_FR = VectorXf::Zero(3);
    VectorXf temp_FL = VectorXf::Zero(3);
    VectorXf temp_RR = VectorXf::Zero(3);
    VectorXf temp_RL = VectorXf::Zero(3);
    MatrixXf temp_total = MatrixXf::Zero(3, 4);

    temp_FR = FR_Foot.G_RefPos;
    temp_FL = FL_Foot.G_RefPos;
    temp_RR = RR_Foot.G_RefPos;
    temp_RL = RL_Foot.G_RefPos;

    VectorXf temp_FR_offset = VectorXf::Zero(3);
    VectorXf temp_FL_offset = VectorXf::Zero(3);

    VectorXf temp_RR_offset = VectorXf::Zero(3);
    VectorXf temp_RL_offset = VectorXf::Zero(3);


    //float future_motion_time = motion_time + (tasktime * (float)k);
    float future_motion_time = com_time + (tasktime * (float)k);
    float newtime = future_motion_time;
    




    // contact 만 여기서 미리 따로 정의를 해보자
    // if(future_motion_time > (walking.time.stance + walking.time.swing)){

    // }
    temp_contact_foots(FL_FootIndex, k) = true;
    temp_contact_foots(FR_FootIndex, k) = true;
    temp_contact_foots(RL_FootIndex, k) = true;
    temp_contact_foots(RR_FootIndex, k) = true;

    // temp_contact_foots.block<4, 1>(0, k) = 


    // 4점 지지 구간
    if(future_motion_time > walking.time.swing && future_motion_time < (walking.time.stance + walking.time.swing)){
        // std::cout << " future_motion_time < stance + swing" << std::endl;
        future_motion_time = walking.time.swing;

        //temp_FR_offset(0) = Function_5thPolyNomial(FR_Foot.X_Coeff, walking.time.swing) - Function_5thPolyNomial(FR_Foot.X_Coeff, 0);
        
        // contact: 4점 지지 조건
        // temp_contact_foots(FL_FootIndex, k) = true;
        // temp_contact_foots(FR_FootIndex, k) = true;
        // temp_contact_foots(RL_FootIndex, k) = true;
        // temp_contact_foots(RR_FootIndex, k) = true;
    }

    else if(future_motion_time >= (walking.time.stance + walking.time.swing)){
        //std::cout << " future_motion_time >= stance + swing" << std::endl;
        future_motion_time -= (walking.time.stance + walking.time.swing);

        flag_StartFootChange = true;

        //newtime = future_motion_time;
        //std::cout << "newtime: " << newtime << std::endl;
        //std::cout << "Startfoot: " << Startfoot << std::endl;

        // Startfoot 이 0일 때... 가 문젠데 //**        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;

        // if(Startfoot == walking.step.START_FOOT_RLFR){
        //     Startfoot = walking.step.START_FOOT_RRFL;
        //     temp_RR_prev = temp_RR;
        //     temp_FL_prev = temp_FL;
        // }
        // else if(Startfoot == walking.step.START_FOOT_RRFL){
        //     Startfoot = walking.step.START_FOOT_RLFR;
        //     temp_FR_prev = temp_FR;
        //     temp_RL_prev = temp_RL;
        // }
        if(Startfoot == walking.step.NONE){
            if(Startfoot_prev == walking.step.START_FOOT_RLFR){
                Startfoot = walking.step.START_FOOT_RRFL;
                
                temp_RR_offset(0) = Function_5thPolyNomial(RR_Foot.X_Coeff, walking.time.swing) - Function_5thPolyNomial(RR_Foot.X_Coeff, 0);
                temp_FL_offset(0) = Function_5thPolyNomial(FL_Foot.X_Coeff, walking.time.swing) - Function_5thPolyNomial(FL_Foot.X_Coeff, 0);
                temp_RR_offset(1) = Function_5thPolyNomial(RR_Foot.Y_Coeff, walking.time.swing) - Function_5thPolyNomial(RR_Foot.Y_Coeff, 0);
                temp_FL_offset(1) = Function_5thPolyNomial(FL_Foot.Y_Coeff, walking.time.swing) - Function_5thPolyNomial(FL_Foot.Y_Coeff, 0);
                // temp_RR_offset(2) = Function_5thPolyNomial(RR_Foot.Z_Coeff, walking.time.swing) - Function_5thPolyNomial(RR_Foot.Z_Coeff, 0);
                // temp_FL_offset(2) = Function_5thPolyNomial(FL_Foot.Z_Coeff, walking.time.swing) - Function_5thPolyNomial(FL_Foot.Z_Coeff, 0);

                temp_RR_offset(2) = Function_5thPolyNomial(RR_Foot.Z_Coeff_Down, walking.time.swing/2.) - Function_5thPolyNomial(RR_Foot.Z_Coeff_Up, 0);
                temp_FL_offset(2) = Function_5thPolyNomial(FL_Foot.Z_Coeff_Down, walking.time.swing/2.) - Function_5thPolyNomial(FL_Foot.Z_Coeff_Up, 0);

              


            }
            else if(Startfoot_prev == walking.step.START_FOOT_RRFL){
                Startfoot = walking.step.START_FOOT_RLFR;

                temp_FR_offset(0) = Function_5thPolyNomial(FR_Foot.X_Coeff, walking.time.swing) - Function_5thPolyNomial(FR_Foot.X_Coeff, 0);
                temp_RL_offset(0) = Function_5thPolyNomial(RL_Foot.X_Coeff, walking.time.swing) - Function_5thPolyNomial(RL_Foot.X_Coeff, 0);
                temp_FR_offset(1) = Function_5thPolyNomial(FR_Foot.Y_Coeff, walking.time.swing) - Function_5thPolyNomial(FR_Foot.Y_Coeff, 0);
                temp_RL_offset(1) = Function_5thPolyNomial(RL_Foot.Y_Coeff, walking.time.swing) - Function_5thPolyNomial(RL_Foot.Y_Coeff, 0);
                
                
                temp_FR_offset(2) = Function_5thPolyNomial(FR_Foot.Z_Coeff_Down, walking.time.swing/2.) - Function_5thPolyNomial(FR_Foot.Z_Coeff_Up, 0);
                
                
                temp_RL_offset(2) = Function_5thPolyNomial(RL_Foot.Z_Coeff_Down, walking.time.swing/2.) - Function_5thPolyNomial(RL_Foot.Z_Coeff_Up, 0);
                // std::cout << "offset: " << temp_RL_offset(2) << std::endl;
                // temp_RL_prev = temp_RL;
            }

            // else if(Startfoot_prev == walking.step.NONE){
            //     temp_FL_offset(0) = FL_Foot.G_RefPos(0);
            //     std::cout << FL_Foot.G_RefPos(0) << std::endl;
            // }
        }

        // 수정중인 변수
        //future_motion_time = walking.time.swing;
        // 이 조건에 들어가고 있질 않다..
        if(Startfoot_prev == walking.step.NONE && Startfoot == walking.step.START_FOOT_RRFL){
                temp_FL_offset(0) = FL_Foot.G_RefPos(0);
                std::cout << FL_Foot.G_RefPos(0) << std::endl;
        }
    }

    

    // "현재 시점의" 스윙 다리가 RLFR 일 때... 그렇기 때문에 이 문장에서는 미래의 스윙다리를 예측하기가 어렵다...
    if (Startfoot       == walking.step.START_FOOT_RLFR){

        if(newtime < walking.time.swing + (float)k*tasktime){
            //temp_contact_foots(FL_FootIndex, k) = true;
            // temp_contact_foots(FR_FootIndex, k) = false;
            // temp_contact_foots(RL_FootIndex, k) = false;
            //temp_contact_foots(RR_FootIndex, k) = true;
        }
        // else if(newtime >= walking.time.swing && newtime < (walking.time.swing + walking.time.stance)){
        //     //temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     //temp_contact_foots(RR_FootIndex, k) = true;

        // }
        // else if(newtime >= (walking.time.swing + walking.time.stance) && newtime < (walking.time.swing + walking.time.stance + walking.time.swing) ){
        //     //temp_contact_foots(FL_FootIndex, k) = false;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     //temp_contact_foots(RR_FootIndex, k) = false;
        // }

        // else if(newtime >= (walking.time.swing + walking.time.stance + walking.time.swing) &&  newtime < 2.*(walking.time.swing + walking.time.stance) ){
        //     //temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     //temp_contact_foots(RR_FootIndex, k) = true;
        // }

        else if(newtime >= 2.*(walking.time.swing + walking.time.stance) + (float)k*tasktime &&  newtime < 2.*(walking.time.swing + walking.time.stance) + walking.time.swing + (float)k*tasktime ){
            //temp_contact_foots(FL_FootIndex, k) = true;
            // temp_contact_foots(FR_FootIndex, k) = false;
            // temp_contact_foots(RL_FootIndex, k) = false;
            //temp_contact_foots(RR_FootIndex, k) = true;
        }


        //temp_FR(AXIS_X)               = Function_5thPolyNomial(FR_Foot.X_Coeff, future_motion_time) + temp_FR_ref(0);
        temp_FR(AXIS_X)               = Function_5thPolyNomial(FR_Foot.X_Coeff, future_motion_time) + temp_FR_offset(AXIS_X);
        // FR_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(FR_Foot.X_Coeff, motion_time);

        temp_FR(AXIS_Y)               = Function_5thPolyNomial(FR_Foot.Y_Coeff, future_motion_time) + temp_FR_offset(AXIS_Y);
        // FR_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(FR_Foot.Y_Coeff, motion_time);
        
        temp_RL(AXIS_X)               = Function_5thPolyNomial(RL_Foot.X_Coeff, future_motion_time) + temp_RL_offset(AXIS_X);
        // RL_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(RL_Foot.X_Coeff, motion_time);

        temp_RL(AXIS_Y)               = Function_5thPolyNomial(RL_Foot.Y_Coeff, future_motion_time) + temp_RL_offset(AXIS_Y);
        // RL_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(RL_Foot.Y_Coeff, motion_time);


        if(future_motion_time < walking.time.swing/2.0){
            temp_FR(AXIS_Z)           = Function_5thPolyNomial(FR_Foot.Z_Coeff_Up, future_motion_time) + temp_FR_offset(AXIS_Z);
            // FR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FR_Foot.Z_Coeff_Up, motion_time);
            
            temp_RL(AXIS_Z)           = Function_5thPolyNomial(RL_Foot.Z_Coeff_Up, future_motion_time) + temp_RL_offset(AXIS_Z);
            // RL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RL_Foot.Z_Coeff_Up, motion_time);

            temp_contact_foots(FR_FootIndex, k) = false;
            temp_contact_foots(RL_FootIndex, k) = false;

        }
        else{
            temp_FR(AXIS_Z)           = Function_5thPolyNomial(FR_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0) + temp_FR_offset(AXIS_Z);
            // FR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FR_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);            
            
            temp_RL(AXIS_Z)           = Function_5thPolyNomial(RL_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0) + temp_RL_offset(AXIS_Z);
            // RL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RL_Foot.Z_Coeff_Down, motion_time - walking.time.swing/2.0);   

            if(future_motion_time < walking.time.swing){
                temp_contact_foots(FR_FootIndex, k) = false;
                temp_contact_foots(RL_FootIndex, k) = false;         
            }
        }        
    }

    else if (Startfoot  == walking.step.START_FOOT_RRFL){

        // if(newtime < walking.time.swing){
        //     temp_contact_foots(FL_FootIndex, k) = false;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = false;            
        // }
        // else if(newtime >= walking.time.swing && newtime < (walking.time.swing + walking.time.stance)){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = true;
        // }
        // else if(newtime >= (walking.time.swing + walking.time.stance) && newtime < (walking.time.swing + walking.time.stance + walking.time.swing) ){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FR_FootIndex, k) = false;
        //     temp_contact_foots(RL_FootIndex, k) = false;
        //     temp_contact_foots(RR_FootIndex, k) = true;
        // }

        // else if(newtime >= (walking.time.swing + walking.time.stance + walking.time.swing) &&  newtime < 2.*(walking.time.swing + walking.time.stance) ){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FR_FootIndex, k) = true;
        //     temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = true;
        // }

        //if(future_motion_time < (walking.time.swing + (float)k*tasktime) && future_motion_time >= 0.){
        if(future_motion_time < walking.time.swing && future_motion_time >= 0.){
            // if(flag_StartFootChange == false){
            //     temp_contact_foots(FL_FootIndex, k) = false;
            //     // temp_contact_foots(FR_FootIndex, k) = false;
            //     // temp_contact_foots(RL_FootIndex, k) = false;
            //     temp_contact_foots(RR_FootIndex, k) = false;
            // }
            // else if(flag_StartFootChange == true){
            //     temp_contact_foots(FL_FootIndex, k) = true;                
            //     temp_contact_foots(RR_FootIndex, k) = true;

            // }
        }
        // else if(newtime >= walking.time.swing && newtime < (walking.time.swing + walking.time.stance)){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     // temp_contact_foots(FR_FootIndex, k) = true;
        //     // temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = true;

        // }
        // else if(newtime >= (walking.time.swing + walking.time.stance) && newtime < (walking.time.swing + walking.time.stance + walking.time.swing) ){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     // temp_contact_foots(FR_FootIndex, k) = true;
        //     // temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = true;
        // }

        // else if(newtime >= (walking.time.swing + walking.time.stance + walking.time.swing) &&  newtime < 2.*(walking.time.swing + walking.time.stance) ){
        //     temp_contact_foots(FL_FootIndex, k) = true;
        //     // temp_contact_foots(FR_FootIndex, k) = true;
        //     // temp_contact_foots(RL_FootIndex, k) = true;
        //     temp_contact_foots(RR_FootIndex, k) = true;
        // }

        // else if(newtime >= 2.*(walking.time.swing + walking.time.stance) + (float)k*tasktime &&  newtime < 2.*(walking.time.swing + walking.time.stance) + walking.time.swing  + (float)k*tasktime){
        //     //temp_contact_foots(FL_FootIndex, k) = true;
        //     temp_contact_foots(FL_FootIndex, k) = false;
        //     temp_contact_foots(RR_FootIndex, k) = false;
        //     //temp_contact_foots(RR_FootIndex, k) = true;
        // }
        
        temp_FL(AXIS_X)               = Function_5thPolyNomial(FL_Foot.X_Coeff, future_motion_time) + temp_FL_offset(AXIS_X);
        // FL_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(FL_Foot.X_Coeff, future_motion_time);

        temp_FL(AXIS_Y)               = Function_5thPolyNomial(FL_Foot.Y_Coeff, future_motion_time) + temp_FL_offset(AXIS_Y);
        // FL_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(FL_Foot.Y_Coeff, future_motion_time);
        
        temp_RR(AXIS_X)               = Function_5thPolyNomial(RR_Foot.X_Coeff, future_motion_time) + temp_RR_offset(AXIS_X);
        // RR_Foot.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(RR_Foot.X_Coeff, future_motion_time);

        temp_RR(AXIS_Y)               = Function_5thPolyNomial(RR_Foot.Y_Coeff, future_motion_time) + temp_RR_offset(AXIS_Y);
        // RR_Foot.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(RR_Foot.Y_Coeff, future_motion_time);
        
        
        if(future_motion_time < walking.time.swing/2.0){

            // if(FL_Foot.Z_Coeff_Up == VectorXf::Zero(6)){
            //     //FL_Foot.Z_Coeff_Up = FR_Foot.Z_Coeff_Up;
            // }
            // FL_Foot.Z_Coeff_Up = FR_Foot.Z_Coeff_Up;


            temp_FL(AXIS_Z)           = Function_5thPolyNomial(FL_Foot.Z_Coeff_Up, future_motion_time) + temp_FL_offset(AXIS_Z);
            // FL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FL_Foot.Z_Coeff_Up, future_motion_time);
            
            temp_RR(AXIS_Z)           = Function_5thPolyNomial(RR_Foot.Z_Coeff_Up, future_motion_time) + temp_RR_offset(AXIS_Z);
            // RR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RR_Foot.Z_Coeff_Up, future_motion_time);

            temp_contact_foots(FL_FootIndex, k) = false;
            temp_contact_foots(RR_FootIndex, k) = false;
        }
        else{            
            temp_FL(AXIS_Z)           = Function_5thPolyNomial(FL_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0) + temp_FL_offset(AXIS_Z);
            // FL_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(FL_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0);            
            
            temp_RR(AXIS_Z)           = Function_5thPolyNomial(RR_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0) + temp_RR_offset(AXIS_Z);
            // RR_Foot.G_RefVel(AXIS_Z)           = Function_5thPolyNomial_dot(RR_Foot.Z_Coeff_Down, future_motion_time - walking.time.swing/2.0);
            if(future_motion_time < walking.time.swing){
                temp_contact_foots(FL_FootIndex, k) = false;
                temp_contact_foots(RR_FootIndex, k) = false;            
            }
        }  
    }

    // 맨 처음 트롯 때, FL 과 RR 의 경우 미래 궤적을 그릴 때 오프셋이 튀는 문제가 있다.
    // 그걸 수정하기 위해 x축에 대한 현재값의 레퍼런스와, 계산된 미래 레퍼런스의 차이가 말도 안되게(0.5) 크면, 현재값의 레퍼런스를 사용토록 수정한다.
    if(Startfoot_prev == walking.step.START_FOOT_RLFR && Startfoot == walking.step.START_FOOT_RRFL){
    
        // std::cout << FL_Foot.G_RefPos(0) << std::endl;
        if(abs(temp_FL(AXIS_X)-FL_Foot.G_RefPos(0)) > 0.2 ){
            temp_FL = FL_Foot.G_RefPos;
        }

        if(abs(temp_RR(AXIS_X)-RR_Foot.G_RefPos(0)) > 0.2 ){
            temp_RR = RR_Foot.G_RefPos;
        }
    }

    temp_total.block<3, 1>(0, 0) = temp_FL;
    temp_total.block<3, 1>(0, 1) = temp_FR;
    temp_total.block<3, 1>(0, 2) = temp_RL;
    temp_total.block<3, 1>(0, 3) = temp_RR;

    if(k == 30){
        temptime = future_motion_time;
    }
    

    return temp_total;
}

VectorXf CRobot::generateCoMTraj_modified(float motion_time, int k) {
    /* generateCoMTraj
     * input : motion_time
     * output : Generate CoM Trajectory
     */

    // float CRobot::Function_5thPolyNomial(Vector6f coeff, float time)
    // com.X_Coeff, 다리.X_Coeff 등 5차 다항식의 계수들은 setWalkingStep 함수 내에서 계산됨
    // motion_time 이 매번 초기화되는 걸 생각하면... Coeff 의 0차 계수가 motion_time=0 일때 y 절편 업데이트 또한 담당하는 건데..

    // int k: 몇 개의 미래를 내다볼 것인지? (k >= 1)

    VectorXf com_RefPosFuture = VectorXf::Zero(3);
    
    //com_motion_time                         = (walking.step.count - walking.step.FINAL_RRFL_SWING_START)*(float)(1.0/onesecScale);
    //walking.step.count += onesecScale*tasktime;

    float future_motion_time = motion_time + (tasktime * (float)k);
    
    Vector6f temp_X_Coeff = com.X_Coeff;
    Vector6f temp_Y_Coeff = com.Y_Coeff;
    Vector6f temp_Z_Coeff = com.Z_Coeff;

    Vector3f com_speed = VectorXf::Zero(3);
    // walking.time.stance                                 = 0.08;
    // walking.time.swing                                  = 0.28;

    if(future_motion_time > (walking.time.stance + walking.time.swing)){
        //future_motion_time -= (walking.time.stance + walking.time.swing);
        
        //future_motion_time -= (walking.time.stance);
        future_motion_time = walking.time.stance + walking.time.swing;

        //temp_X_Coeff(0) += (com.G_RefTargetPos(0)-com.G_RefPrePos(0));
        com_speed(0) = (com.G_RefTargetPos(0)-com.G_RefPrePos(0))/(walking.time.stance+walking.time.swing);
        com_speed(1) = (com.G_RefTargetPos(1)-com.G_RefPrePos(1))/(walking.time.stance+walking.time.swing);
        com_speed(2) = (com.G_RefTargetPos(2)-com.G_RefPrePos(2))/(walking.time.stance+walking.time.swing);

    }

        
    //com_RefPosFuture(AXIS_X)               = Function_5thPolyNomial(temp_X_Coeff, future_motion_time) + com_speed*(motion_time + (tasktime * (float)k) - (walking.time.stance + walking.time.swing));
    com_RefPosFuture(AXIS_X)               = Function_5thPolyNomial(temp_X_Coeff, future_motion_time) + com_speed(0)*(motion_time + (tasktime * (float)k) - (walking.time.stance + walking.time.swing));
    //com.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(com.X_Coeff, motion_time);
    //com.G_RefAcc(AXIS_X)               = Function_5thPolyNomial_2dot(com.X_Coeff, motion_time);

    com_RefPosFuture(AXIS_Y)               = Function_5thPolyNomial(temp_Y_Coeff, future_motion_time) + com_speed(1)*(motion_time + (tasktime * (float)k) - (walking.time.stance + walking.time.swing));
    //com.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(com.Y_Coeff, motion_time);
    //com.G_RefAcc(AXIS_Y)               = Function_5thPolyNomial_2dot(com.Y_Coeff, motion_time);

    com_RefPosFuture(AXIS_Z)               = Function_5thPolyNomial(temp_Z_Coeff, future_motion_time) + com_speed(2)*(motion_time + (tasktime * (float)k) - (walking.time.stance + walking.time.swing));
    //com.G_RefVel(AXIS_Z)               = Function_5thPolyNomial_dot(com.Z_Coeff, motion_time);
    //com.G_RefAcc(AXIS_Z)               = Function_5thPolyNomial_2dot(com.Z_Coeff, motion_time);

    // base.G_RefOri(AXIS_ROLL)           = Function_5thPolyNomial(base.Roll_Coeff, motion_time);
    // base.G_RefOri(AXIS_PITCH)          = Function_5thPolyNomial(base.Pitch_Coeff, motion_time);
    // base.G_RefOri(AXIS_YAW)            = Function_5thPolyNomial(base.Yaw_Coeff, motion_time);

    // base.G_RefAngularVel(AXIS_ROLL)    = Function_5thPolyNomial_dot(base.Roll_Coeff, motion_time);
    // base.G_RefAngularVel(AXIS_PITCH)   = Function_5thPolyNomial_dot(base.Pitch_Coeff, motion_time);
    // base.G_RefAngularVel(AXIS_YAW)     = Function_5thPolyNomial_dot(base.Yaw_Coeff, motion_time);
    
    // base.G_RefAngularAcc(AXIS_ROLL)    = Function_5thPolyNomial_2dot(base.Roll_Coeff, motion_time);
    // base.G_RefAngularAcc(AXIS_PITCH)   = Function_5thPolyNomial_2dot(base.Pitch_Coeff, motion_time);
    // base.G_RefAngularAcc(AXIS_YAW)     = Function_5thPolyNomial_2dot(base.Yaw_Coeff, motion_time);

    return com_RefPosFuture;
}


void CRobot::OptimizationProcess(const VectorXf& com_target,\
                                 float& stance_time, float& swing_time,\
                                 float& Xstep, float& Ystep, float& Turnstep)
{
    // 입력 com_target: 조이스틱을 통해 받은? 목표 속도(x, y, psi)
    // 출력 스탠스 속도, 스윙 속도, 보폭... (현재는 트롯 알고리즘 기준인데, 추후 트롯과 워크에 대해서 다 뽑아주는 방식으로 변경하면 문제 없을 것 같다?)


    // 0. Trot::Init 의 내용 -> 스탠스 시간과 스윙 시간이 현재는 알고리즘 없이 상수로 pre-defined 됐음.. 추후 변경하는 로직을 작성해야 함
    walking.time.ready                                  = 0.10;         // trot 초기 실행 시 살짝 경직되는 시간인 듯 하다.
    walking.time.ready  = 0.;
        
    walking.time.stance                                 = 0.08;
    walking.time.swing                                  = 0.28;



    

    // 1. Flag 를 검사하고 있음            
    if (walking.Ready == false){
        return;
    }
    
    if (walking.UpDown == false){
        cout << "[PongBot] : Trot Mode can't activate ! " << endl;
        ControlMode                                     = CTRLMODE_WALK_READY;
        CommandFlag                                     = STAND;
        return;
    }
    
    if(walking.MoveDone == true){
        walking.MoveDone                                = false;
        walking.step.count                              = 0;
        cout << "[PongBot] : Trot Mode" << endl;
    }
    
    // 2. count 를 검사, 0  일 시 Trot initialize 를 해준 후 phase 플래그를 바꾼다
    if (walking.step.count == 0) {
        Trot_init();
        walking.phase                                   = TROT_READY;
    }
    

    // 3. walking.step.count 가 특정 시간일 때 마다 walking.phase 가 변경돼고
    // 보아하니 walking.step.count 는 720(0) 부터 720 까지 반복
    // 720 에서 다시 초기화돼는 이유는, TROT_STANCE 에서 walking.stop 이 FALSE 이기 때문에, 여기서 0으로 초기화시켜준다.
    // 만약 사용자가 동그라미 버튼을 통해 trot stop 을 실행할 경우 walking.stop 이 활성화돼며, 카운트는 720~1440 에 도달하게 된다.
    if (walking.step.count == walking.step.RLFR_SWING_START + onesecScale*tasktime) {
        
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;
        
        //** 미래 궤적을 설계할 때, start_foot 이 -1, 0, 1 만으로는 표현하기 어려움을 느낌.
        //** 0일때 이전이 -1이었는지 1이었는지 표현해주는 것이 필요
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;


    }
    else if (walking.step.count == walking.step.RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        setWalkingSpeedLimit();
        
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;
                        
        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
        
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RLFR;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RLFR_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                            
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RLFR;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_SWING_START + onesecScale*tasktime) {
        walking.phase                                   = TROT_SWING;
        walking.step.start_foot                         = walking.step.START_FOOT_RRFL;
        
        walking.MovingSpeed                             << 0, 0, 0;
        walking.step.FBstepSize                         = (walking.PreMovingSpeed(AXIS_X)   + walking.MovingSpeed(AXIS_X))   * walking.time.step;
        walking.step.LRstepSize                         = (walking.PreMovingSpeed(AXIS_Y)   + walking.MovingSpeed(AXIS_Y))   * walking.time.step;
        walking.step.TurnSize                           = (walking.PreMovingSpeed(AXIS_YAW) + walking.MovingSpeed(AXIS_YAW)) * walking.time.step;

        setWalkingStep(walking.step.FBstepSize, walking.step.LRstepSize, walking.step.TurnSize, walking.step.start_foot);
        
        walking.PreMovingSpeed(AXIS_X)                  = walking.MovingSpeed(AXIS_X);
        walking.PreMovingSpeed(AXIS_Y)                  = walking.MovingSpeed(AXIS_Y);
        walking.PreMovingSpeed(AXIS_YAW)                = walking.MovingSpeed(AXIS_YAW);
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_START + onesecScale*tasktime) {
        contact.Foot[FL_FootIndex]                      = true;
        contact.Foot[FR_FootIndex]                      = true;
        contact.Foot[RL_FootIndex]                      = true;
        contact.Foot[RR_FootIndex]                      = true;
                             
        walking.phase                                   = TROT_STANCE;
        walking.step.start_foot                         = walking.step.NONE;

        //**
        walking.step.start_foot_prev = walking.step.START_FOOT_RRFL;
    }
    else if (walking.step.count == walking.step.FINAL_RRFL_STANCE_END + onesecScale*tasktime) {
        CommandFlag                                     = NONE_ACT;
        walking.MoveDone                                = true;
        std::cout << "come here??? count: " << walking.step.count << std::endl;
    }




    //


    std::cout << "ref speed: \n" << com_target << std::endl;
    //std::cout << "walk_para.time.swing: \n" << walk_para.time.swing << std::endl;
    //walk_para.MovingSpeed = VectorXf::Zero(3);




    // 출력 1.
    stance_time = walking.time.stance;
    swing_time = walking.time.swing;
    Xstep = walking.step.FBstepSize;
    Ystep = walking.step.LRstepSize;
    Turnstep = walking.step.TurnSize;
}

void CRobot::ChooseWalkOrTrot(const float& stance_time, const float& swing_time, const float& Xstep, const float& Ystep, const float& Turnstep,\
                              int& walktype)    // 추후 만들어야 하는 내용
{
    // x방향 보폭, y방향 보폭, 회전 보폭, 스탠스 시간과 스윙 시간을 받아서 어떤 걸음새를 하겠다고 설정하는 알고리즘을 설계해서 넣어야함
    walktype = TROTMODE;
}


void CRobot::CoMTrajectory(const int& walktype, const VectorXf& com_target_modified, Vector3f& com_ref)
{
    switch(walktype){
        case TROTMODE:
        // 1. com_target_modified 를 이용한 궤적을 만들건데,
        // 현재 퐁봇의 궤적은 다음과 같은 알고리즘으로 생성
        // 목표 속도 -> 5차 다항식에 사용할 계수를 nrutil 사용하여 다리, com 각각의 계수 만듬 -> 사용한 계수와 시간을 이용하여 com 또는 다리의 목표 위치 출력
        // com 계수는 setWalkingStep 함수에서 만들어주는데, 필요한 내용을 발췌해서 와야할 것 같다
        calculateCoM5thPolynomialCoeff();


        break;


        case WALKMODE:
        // 현재는 공백

        break;
    }

}


























void CRobot::generateCoMTraj_new(float motion_time) {
    /* generateCoMTraj
     * input : motion_time
     * output : Generate CoM Trajectory
     */

    // 현재 변경하고 싶은 함수의 형태로는 generateCoMTraj_new(Vector3f CoM_vel, int WalkType, COM& com)
    // 이 되야할 듯 한데,
    // CoM_vel 을 받아서 com.X_Coeff 를 계산하는 내용이 여기 있어야 할 것이다.. 5차 다항식의 계수로 구성된 6x1 벡터이다.
    // 실제로 com.X_Coeff 를 계산해주는 알고리즘은 


    com.G_RefPos(AXIS_X)               = Function_5thPolyNomial(com.X_Coeff, motion_time);
    com.G_RefVel(AXIS_X)               = Function_5thPolyNomial_dot(com.X_Coeff, motion_time);
    com.G_RefAcc(AXIS_X)               = Function_5thPolyNomial_2dot(com.X_Coeff, motion_time);

    com.G_RefPos(AXIS_Y)               = Function_5thPolyNomial(com.Y_Coeff, motion_time);
    com.G_RefVel(AXIS_Y)               = Function_5thPolyNomial_dot(com.Y_Coeff, motion_time);
    com.G_RefAcc(AXIS_Y)               = Function_5thPolyNomial_2dot(com.Y_Coeff, motion_time);

    com.G_RefPos(AXIS_Z)               = Function_5thPolyNomial(com.Z_Coeff, motion_time);
    com.G_RefVel(AXIS_Z)               = Function_5thPolyNomial_dot(com.Z_Coeff, motion_time);
    com.G_RefAcc(AXIS_Z)               = Function_5thPolyNomial_2dot(com.Z_Coeff, motion_time);

    base.G_RefOri(AXIS_ROLL)           = Function_5thPolyNomial(base.Roll_Coeff, motion_time);
    base.G_RefOri(AXIS_PITCH)          = Function_5thPolyNomial(base.Pitch_Coeff, motion_time);
    base.G_RefOri(AXIS_YAW)            = Function_5thPolyNomial(base.Yaw_Coeff, motion_time);

    base.G_RefAngularVel(AXIS_ROLL)    = Function_5thPolyNomial_dot(base.Roll_Coeff, motion_time);
    base.G_RefAngularVel(AXIS_PITCH)   = Function_5thPolyNomial_dot(base.Pitch_Coeff, motion_time);
    base.G_RefAngularVel(AXIS_YAW)     = Function_5thPolyNomial_dot(base.Yaw_Coeff, motion_time);
    
    base.G_RefAngularAcc(AXIS_ROLL)    = Function_5thPolyNomial_2dot(base.Roll_Coeff, motion_time);
    base.G_RefAngularAcc(AXIS_PITCH)   = Function_5thPolyNomial_2dot(base.Pitch_Coeff, motion_time);
    base.G_RefAngularAcc(AXIS_YAW)     = Function_5thPolyNomial_2dot(base.Yaw_Coeff, motion_time);
}


// setWalkingStep 함수에 있는 내용들을 분리해보려고 하는 중
// void CRobot::calculateCoM5thPolynomialCoeff(Vector6f& XCoeff, Vector6f& YCoeff, Vector6f& ZCoeff, )
// {
//     float** Base_TargetRotMat                       = matrix(1, 3, 1, 3);
//     float** tempOutMat                              = matrix(1, 3, 1, 3);
    
//     float** tempOutMat66                            = matrix(1, 6, 1, 6);
//     float** OutMat66                                = matrix(1, 6, 1, 6);
    
//     float* tempOutVec6                              = fvector(1, 6);
//     float* OutVec6                                  = fvector(1, 6);
    
//     float* MoveSize                                 = fvector(1, 3);
//     float* PrePosition                              = fvector(1, 3);
    
//     float* OutVec                                   = fvector(1, 3);
//     float* tempOutVec                               = fvector(1, 3);


//     com.G_RefPrePos                                     = com.G_RefPos;
//     com.G_RefPreVel                                     = com.G_RefVel;
//     com.G_RefPreAcc                                     = com.G_RefAcc;

//     for (int AXIS = 0; AXIS < 3; ++AXIS) {
//         PrePosition[AXIS + 1]                           = com.G_RefPrePos(AXIS);
//         MoveSize[AXIS + 1]                              = walking.step.com.MoveSize(AXIS);
//     }
//     mvmult(Base_TargetRotMat, 3, 3, MoveSize, 3, tempOutVec);  
//     vadd(PrePosition, 3, tempOutVec, OutVec);
//     for (int AXIS = 0; AXIS < 3; ++AXIS) {
//         com.G_RefTargetPos(AXIS)                      = OutVec[AXIS + 1];
//     }
    
//     com.G_RefTargetVel(AXIS_X)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_X);
//     com.G_RefTargetVel(AXIS_Y)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_Y);
//     com.G_RefTargetVel(AXIS_Z)               = ((com.G_RefTargetPos - com.G_RefPrePos)/walking.time.step)(AXIS_Z);    


//     for (int i = 0; i < 3; ++i) {
//         walking.PreState                <<    com.G_RefPrePos(i),    com.G_RefPreVel(i),    com.G_RefPreAcc(i);
//         walking.TargetState             << com.G_RefTargetPos(i), com.G_RefTargetVel(i), com.G_RefTargetAcc(i);

//         if (i == AXIS_X){
//             set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
//             inverse(tempOutMat66, 6, OutMat66);
//             for (int AXIS = 0; AXIS < 3; ++AXIS) {
//                 tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
//                 tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
//             }
//             mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
//             for (int AXIS = 0; AXIS < 6; ++AXIS) {
//                 com.X_Coeff[AXIS]                        = OutVec6[AXIS + 1];
//             }            
//         }
//         else if (i == AXIS_Y){
//             set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
//             inverse(tempOutMat66, 6, OutMat66);
//             for (int AXIS = 0; AXIS < 3; ++AXIS) {
//                 tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
//                 tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
//             }
//             mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
//             for (int AXIS = 0; AXIS < 6; ++AXIS) {
//                 com.Y_Coeff[AXIS]                        = OutVec6[AXIS + 1];
//             }            
//         }
//         else if (i == AXIS_Z){        
//             set_5thPolyNomialMatrix(walking.time.step, tempOutMat66);
//             inverse(tempOutMat66, 6, OutMat66);
//             for (int AXIS = 0; AXIS < 3; ++AXIS) {
//                 tempOutVec6[2*AXIS + 1]                       = walking.PreState(AXIS);
//                 tempOutVec6[2*AXIS + 2]                       = walking.TargetState(AXIS);  
//             }
//             mvmult(OutMat66, 6, 6, tempOutVec6, 6, OutVec6);
            
//             for (int AXIS = 0; AXIS < 6; ++AXIS) {
//                 com.Z_Coeff[AXIS]                        = OutVec6[AXIS + 1];
//             }            
//         }
//     }

//     free_matrix(Base_TargetRotMat, 1, 3, 1, 3);
//     free_matrix(tempOutMat, 1, 3, 1, 3);
//     free_vector(MoveSize, 1, 3);
//     free_vector(PrePosition, 1, 3);
//     free_vector(OutVec, 1, 3);
//     free_vector(tempOutVec, 1, 3);    
    
//     free_matrix(tempOutMat66, 1, 6, 1, 6);
//     free_matrix(OutMat66, 1, 6, 1, 6);
//     free_vector(tempOutVec6, 1, 6);
//     free_vector(OutVec6, 1, 6);    
// }


#if 0
void CRobot::OSQP_Init(void) {
    //    c_int A_p_num = 13 * horizontal_length;
    /* OSQP_Init
     * Initialize OSQP Setting
     */
    // const int RL_Foot_AXIS_Z = BaseDOF + (AXIS_Z + RL_FootIndex * FootDOF);
    // const int RR_Foot_AXIS_Z = BaseDOF + (AXIS_Z + RR_FootIndex * FootDOF);
    // const int FL_Foot_AXIS_Z = BaseDOF + (AXIS_Z + FL_FootIndex * FootDOF);
    // const int FR_Foot_AXIS_Z = BaseDOF + (AXIS_Z + FR_FootIndex * FootDOF);

    osqp.com.RefAcc << 0, 0, 0;
    osqp.base.RefAngularAcc << 0, 0, 0;

    osqp.Mass = systemID.RobotMass;

    osqp.I_g << 1.84, 0, 0,
            0, 5.34, 0,
            0, 0, 6.03;




    osqp.Gravity << 0, 0, 9.81;

    osqp.b << osqp.Mass * (osqp.com.RefAcc + osqp.Gravity),
            osqp.I_g * osqp.base.RefAngularAcc;


    // WsJi Gain

    gain.CoM_QP_Kp << 300, 0, 0,
            0, 300, 0,
            0, 0, 300;

    gain.CoM_QP_Kd << 30, 0, 0,
            0, 30, 0,
            0, 0, 30;




    gain.BASE_QP_Kp << 3000, 0, 0,
            0, 3000, 0,
            0, 0, 3000;

    gain.BASE_QP_Kd << 500, 0, 0,
            0, 500, 0,
            0, 0, 500;

    //** present state and reference states

    //** Define Weighting Matrix
    //MatrixXd L_mpc                          = MatrixXd::Identity(13*horizontal_length, 13*horizontal_length);
    MatrixXf L_ori = MatrixXf::Identity(3, 3);
    MatrixXf L_pos = MatrixXf::Identity(3, 3);
    MatrixXf L_d_ori = MatrixXf::Identity(3, 3);
    MatrixXf L_d_pos = MatrixXf::Identity(3, 3);

    double gain_g_ = 1;
 
    L_ori << 500000000, 0, 0,
            0, 500000000, 0,
            0, 0, 500000000;
    //    L_ori << 1000000000, 0, 0,
    //            0, 1000000000, 0,
    //            0, 0, 200000000;

    L_pos << 3000000000, 0, 0,
            0, 3000000000, 0,
            0, 0, 7000000000;

    L_d_ori << 50000, 0, 0,
            0, 150000, 0,
            0, 0, 150000;
    //    L_d_ori << 100000, 0, 0,
    //            0, 100000, 0,
    //            0, 0, 50000;

    L_d_pos << 70000, 0, 0,
            0, 70000, 0,
            0, 0, 70000;

    osqp.K_mpc *= 0.1; // 0.04 보단 작아야겠다. 0.035

    //L_pos *= 2.;    
    //L_d_pos *= 8.;

    // L_pos *= 8.;
    L_pos *= 15.;
    
    L_d_pos *= 10.;
    
    // L_ori *= 5.;
    L_ori *= 10.;
    L_ori(1,1) *= 3.;
    L_d_ori *= 2.;

    

    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.L_mpc.block<3, 3>(i * 13, i * 13) = L_ori;
        osqp.L_mpc.block<3, 3>(i * 13 + 3, i * 13 + 3) = L_pos;
        osqp.L_mpc.block<3, 3>(i * 13 + 6, i * 13 + 6) = L_d_ori;
        osqp.L_mpc.block<3, 3>(i * 13 + 9, i * 13 + 9) = L_d_pos;
        osqp.L_mpc(i * 13 + 12, i * 13 + 12) = gain_g_;
    }


    //osqp.L_mpc *= 100000.;


    //** Define A_c

    osqp.A_c.block<3, 3>(0, 6) = MatrixXf::Identity(3, 3); // Rot_z(psi), assume that psi = 0
    osqp.A_c.block<3, 3>(3, 9) = MatrixXf::Identity(3, 3); // Identity(3,3) on <4, 
    osqp.A_c(11, 12) = -1;

    osqp.POS_CoM_2_RL_CrossProduct << 0, -RL_Foot.RefPos_CoM_local(AXIS_Z), RL_Foot.RefPos_CoM_local(AXIS_Y),
            RL_Foot.RefPos_CoM_local(AXIS_Z), 0, -RL_Foot.RefPos_CoM_local(AXIS_X),
            -RL_Foot.RefPos_CoM_local(AXIS_Y), RL_Foot.RefPos_CoM_local(AXIS_X), 0;

    osqp.POS_CoM_2_RR_CrossProduct << 0, -RR_Foot.RefPos_CoM_local(AXIS_Z), RR_Foot.RefPos_CoM_local(AXIS_Y),
            RR_Foot.RefPos_CoM_local(AXIS_Z), 0, -RR_Foot.RefPos_CoM_local(AXIS_X),
            -RR_Foot.RefPos_CoM_local(AXIS_Y), RR_Foot.RefPos_CoM_local(AXIS_X), 0;

    osqp.POS_CoM_2_FL_CrossProduct << 0, -FL_Foot.RefPos_CoM_local(AXIS_Z), FL_Foot.RefPos_CoM_local(AXIS_Y),
            FL_Foot.RefPos_CoM_local(AXIS_Z), 0, -FL_Foot.RefPos_CoM_local(AXIS_X),
            -FL_Foot.RefPos_CoM_local(AXIS_Y), FL_Foot.RefPos_CoM_local(AXIS_X), 0;

    osqp.POS_CoM_2_FR_CrossProduct << 0, -FR_Foot.RefPos_CoM_local(AXIS_Z), FR_Foot.RefPos_CoM_local(AXIS_Y),
            FR_Foot.RefPos_CoM_local(AXIS_Z), 0, -FR_Foot.RefPos_CoM_local(AXIS_X),
            -FR_Foot.RefPos_CoM_local(AXIS_Y), FR_Foot.RefPos_CoM_local(AXIS_X), 0;

    osqp.B_c.block<3, 3>(6, 0) = osqp.I_g.inverse() * osqp.POS_CoM_2_RL_CrossProduct;
    //    osqp.B_c.block<3, 3>(6, 0) = MatrixXd::Zero(3, 3);
    osqp.B_c.block<3, 3>(6, 3) = osqp.I_g.inverse() * osqp.POS_CoM_2_RR_CrossProduct;
    osqp.B_c.block<3, 3>(6, 6) = osqp.I_g.inverse() * osqp.POS_CoM_2_FL_CrossProduct;
    osqp.B_c.block<3, 3>(6, 9) = osqp.I_g.inverse() * osqp.POS_CoM_2_FR_CrossProduct;

    osqp.B_c.block<3, 3>(9, 0) = (1 / osqp.Mass) * MatrixXd::Identity(3, 3);
    //    osqp.B_c.block<3, 3>(9, 0) = (1 / osqp.Mass) * MatrixXd::Zero(3, 3);
    osqp.B_c.block<3, 3>(9, 3) = (1 / osqp.Mass) * MatrixXd::Identity(3, 3);
    osqp.B_c.block<3, 3>(9, 6) = (1 / osqp.Mass) * MatrixXd::Identity(3, 3);
    osqp.B_c.block<3, 3>(9, 9) = (1 / osqp.Mass) * MatrixXd::Identity(3, 3);

    //** Discrete Time
    osqp.A_hat = osqp.A_c * dt + MatrixXd::Identity(13, 13);

    osqp.B_hat = osqp.B_c * dt;


    //** Define A_qp
    MatrixXd A_temp = osqp.A_hat;
    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.A_qp.block<13, 13>(i * 13, 0) = A_temp;
        A_temp *= osqp.A_hat;
    }

    //** Predefine A_hat_squared
    //** Calculate CoM
#if 0
    myCOM = 0.25 * EulerZYX(base.CurrentOri(2), base.CurrentOri(1), base.CurrentOri(0))\
            *(-RL_Foot.CurrentPos_local - RR_Foot.CurrentPos_local - FL_Foot.CurrentPos_local - FR_Foot.CurrentPos_local);

    /*myCOM = 0.33*EulerZYX(base.CurrentOri(2), base.CurrentOri(1), base.CurrentOri(0))\
     *(-RL_Foot.CurrentPos_local - RR_Foot.CurrentPos_local - FR_Foot.CurrentPos_local);
     */


    std::cout << "myCOM:\n" << myCOM << std::endl;

    //Vector3d myCOM_ref;
    //bool testflag = true;
    if (testflag) {
        myCOM_init = myCOM;
        /*myCOM_init = 0.33*EulerZYX(base.CurrentOri(2), base.CurrentOri(1), base.CurrentOri(0))\
         *(-RL_Foot.CurrentPos_local - RR_Foot.CurrentPos_local - FR_Foot.CurrentPos_local);
        
        myCOM_init(1) -= 0.08;*/


        testflag = false;
    }

    myCOM_ref = myCOM_init;

    std::cout << "myCOM_ref:\n" << myCOM_ref << std::endl;
    std::cout << "myCOM_init:\n " << myCOM_init << std::endl;
#endif

    //** X states(vector, 13x1): 
    // 1. RPY
    osqp.X_mpc(0) = base.CurrentOri(0);
    osqp.X_mpc(1) = base.CurrentOri(1);
    osqp.X_mpc(2) = base.CurrentOri(2);

    // 2. Robot pos
    osqp.X_mpc(3) = myCOM(0);
    osqp.X_mpc(4) = myCOM(1);
    osqp.X_mpc(5) = myCOM(2);

    // 3. Angular speed
    osqp.X_mpc(6) = base.CurrentAngularVel(0);
    osqp.X_mpc(7) = base.CurrentAngularVel(1);
    osqp.X_mpc(8) = base.CurrentAngularVel(2);

    // 4. Robot vel (in this time, vel should be zero)
    osqp.X_mpc(9) = myCOM(0);
    osqp.X_mpc(10) = myCOM(1);
    osqp.X_mpc(11) = myCOM(2);

    // 5. Robot 
    osqp.X_mpc(12) = osqp.Gravity(2);

    //** X reference states(vector, (13*horizon)*1)
    // COM ref trajectory should be designed
    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.X_ref_horizon.block(i * 13, 0, 13, 1) = osqp.X_mpc; //일단은 그냥 시작위치를 미래의 관측위치로 계속 두어도 상관이 없지 않을까 싶으요?
        //osqp.X_ref_horizon.block(13, 0, 13, 1) = osqp.X_mpc;
        //osqp.X_ref_horizon.block(26, 0, 13, 1) = osqp.X_mpc;
    }




    //** Define B_qp
    // 이제 저 아래의 B_hat 도 B_temp 를 이용하고, 추후 다리를 드는 모션에서는 B_temp matrix 를 변동한다던지 하면..?

    for (size_t m = 0; m < horizontal_length; ++m) { // row
        osqp.B_qp.block<13, 12>(m * 13, m * 12) = osqp.B_hat;
        A_temp = osqp.A_hat;
        for (size_t n = 0; n < horizontal_length; ++n) { //column
            //osqp.B_qp.block<13, 12>(m*13, n*12) = osqp.B_hat;

            if (m > n) {
                osqp.B_qp.block<13, 12>(m * 13, (m - n - 1)*12) = A_temp * osqp.B_hat;
                A_temp *= osqp.A_hat;
            } else if (m == n) {
                break;
            }

        }
    }

    /*
    osqp.POS_CoM_CrossProduct   <<                                      0,      -RL_Foot.RefPos_CoM_local(AXIS_Z),        RL_Foot.RefPos_CoM_local(AXIS_Y),                                       0,       -RR_Foot.RefPos_CoM_local(AXIS_Z),        RR_Foot.RefPos_CoM_local(AXIS_Y),                                        0,       -FL_Foot.RefPos_CoM_local(AXIS_Z),          FL_Foot.RefPos_CoM_local(AXIS_Y),                                       0,        -FR_Foot.RefPos_CoM_local(AXIS_Z),          FR_Foot.RefPos_CoM_local(AXIS_Y),
                                        RL_Foot.RefPos_CoM_local(AXIS_Z),                                       0,       -RL_Foot.RefPos_CoM_local(AXIS_X),       RR_Foot.RefPos_CoM_local(AXIS_Z),                                        0,       -RR_Foot.RefPos_CoM_local(AXIS_X),        FL_Foot.RefPos_CoM_local(AXIS_Z),                                        0,         -FL_Foot.RefPos_CoM_local(AXIS_X),       FR_Foot.RefPos_CoM_local(AXIS_Z),                                         0,         -FR_Foot.RefPos_CoM_local(AXIS_X),
                                       -RL_Foot.RefPos_CoM_local(AXIS_Y),       RL_Foot.RefPos_CoM_local(AXIS_X),                                        0,      -RR_Foot.RefPos_CoM_local(AXIS_Y),        RR_Foot.RefPos_CoM_local(AXIS_X),                                        0,       -FL_Foot.RefPos_CoM_local(AXIS_Y),        FL_Foot.RefPos_CoM_local(AXIS_X),                                          0,      -FR_Foot.RefPos_CoM_local(AXIS_Y),         FR_Foot.RefPos_CoM_local(AXIS_X),                                          0;
 
     */


    //osqp.A.block<3, 12>(3, 0)   =   osqp.POS_CoM_CrossProduct;



    /*
    osqp.P =  osqp.A.transpose() * osqp.S * osqp.A + osqp.alpha * osqp.W;
    osqp.q = -osqp.A.transpose() * osqp.S * osqp.b;
     */

    // WsJi
    //osqp.P =  2.*((osqp.A.transpose() * osqp.S * osqp.A) + (osqp.alpha * osqp.W * osqp.alpha));
    //osqp.q = -2.* osqp.b.transpose() * osqp.S * osqp.A;


    osqp.P_mpc = 2 * (osqp.B_qp.transpose() * osqp.L_mpc * osqp.B_qp + osqp.K_mpc);
    osqp.q_mpc = 2 * osqp.B_qp.transpose() * osqp.L_mpc * (osqp.A_qp * osqp.X_mpc - osqp.X_ref_horizon);



    //std::cout << "A" << std::endl;

    // ===================== OSQP  ====================== //
    int row_index = 0;
    int row_index_max = 0;
    int col_index = 0;

    // ===================== P_x ====================== //
    //    c_int P_nnz = 0;
    int OSQP_num1 = 0;
    int OSQP_num2 = 0;
    int OSQP_num3 = 0;

    //    for (int j = 0; j <= osqp.P_mpc.cols() - 1; ++j) {
    //        for (int i = 0; i <= j; ++i) {
    //            if (osqp.P_mpc(i, j) != 0) {
    //                P_nnz++;
    //            }
    //        }
    //    }
    //    c_float OSQP_P_x[P_nnz];
    //    c_int OSQP_P_i[P_nnz];
    //    c_int OSQP_P_p[osqp.P_mpc.cols() + 1];
    // 5. osqp QP form 의 P에 대한 행렬들을 위에서 만든 osqp.P 행렬을 이용하여 만들어줄 것(희소행렬로 표현해주기 위한 성분들을 만들 것)
    // ===================== P_x ====================== //    
    //    osqp.data_P_x[]
    for (int j = 0; j < osqp.P_mpc.cols(); ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i <= j; ++i) {
            if (osqp.P_mpc(i, j) != 0) {
                osqp.P_x_mpc[OSQP_num1] = osqp.P_mpc(i, j);
                osqp.P_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.P_p_mpc[j + 1] = OSQP_num3;
    }
    //    std::cout << "B" << std::endl;
    // ===================== P_i ====================== //
    //osqp.data_P_i[]


    // ===================== P_p ====================== //
    //osqp.data_P_p[]

    // ===================== q ====================== //
    //osqp.data_q[i]
    for (unsigned int i = 0; i < osqp.P.cols(); ++i) {
        osqp.data_q[i] = osqp.q(i);
    }
    //    c_float q_1[horizontal_length * 12];
    // ===================== q ====================== //
    for (unsigned int i = 0; i < horizontal_length * 12; ++i) {
        osqp.q_mpc_[i] = osqp.q_mpc(i);
    }
    //    std::cout << "C" << std::endl;


    //* WsJi - A Matrix prepare
    Vector3d n_(0, 0, 1);
    Vector3d t1_(1, 0, 0);
    Vector3d t2_(0, 1, 0);

    double fric_coef = 0.8;

    MatrixXd C1_(5, 3), C2_(5, 3), C3_(5, 3), C4_(5, 3);
    C1_.block(0, 0, 1, 3) = (-fric_coef * n_ + t1_).transpose();
    C1_.block(1, 0, 1, 3) = (-fric_coef * n_ + t2_).transpose();
    C1_.block(2, 0, 1, 3) = (fric_coef * n_ + t2_).transpose();
    C1_.block(3, 0, 1, 3) = (fric_coef * n_ + t1_).transpose();
    C1_.block(4, 0, 1, 3) = (n_).transpose();

    C2_ = C1_;
    C3_ = C1_;
    C4_ = C1_;

    osqp.C_.block(0, 0, 5, 3) = C1_;
    osqp.C_.block(5, 3, 5, 3) = C2_;
    osqp.C_.block(10, 6, 5, 3) = C3_;
    osqp.C_.block(15, 9, 5, 3) = C4_;


    //** MPC horizontal for osqp.C
    for (size_t i = 0; i < horizontal_length; ++i) {
        osqp.C_mpc.block(i * 20, i * 12, 20, 12) = osqp.C_;
    }

    //
    /*
    c_int mA_nnz;

    std::vector<double> mA_x;
    std::vector<int> mA_i, mA_p;

    unsigned int mIndex = 0;


    for(size_t j = 0; j < osqp.C_.cols(); ++j){
        bool mTemp = false;
        

        for(size_t i = 0; i < osqp.C_.rows(); ++i){

            //std::cout << C_(i,j) << std::endl;
            if(osqp.C_(i,j) != 0){
                
                mA_x.push_back(osqp.C_(i,j));
                mA_i.push_back(i);
                if(!mTemp){
                    mTemp = true;
                    mA_p.push_back(mIndex);
                }
                ++mIndex;
            }
        }
        mTemp = false;
    }    

    mA_nnz = mA_x.size();
    mA_p.push_back(mA_nnz);

    osqp.data_A_nnz = mA_nnz;
     */

    //    c_int A_nnz = 0;
    //    for (int i = 0; i < 20 * horizontal_length; ++i) {
    //        for (int j = 0; j < 12 * horizontal_length; ++j) {
    //            if (osqp.C_mpc(i, j) != 0) {
    //                A_nnz++;
    //            }
    //        }
    //    }

    //    c_float OSQP_A_x[A_nnz];
    //    c_int OSQP_A_i[A_nnz];
    //        std::cout << "P_nnz = " << P_nnz << std::endl;
    //        std::cout << "A_nnz = " << A_nnz << std::endl;
    OSQP_num1 = 0;
    OSQP_num2 = 0;
    OSQP_num3 = 0;
    for (int j = 0; j < 12 * horizontal_length; ++j) {
        OSQP_num2 = 0;
        for (int i = 0; i < 20 * horizontal_length; ++i) {
            if (osqp.C_mpc(i, j) != 0) {
                osqp.A_x_mpc[OSQP_num1] = osqp.C_mpc(i, j);
                osqp.A_i_mpc[OSQP_num1] = i;
                OSQP_num2++;
                OSQP_num1++;
            }
        }
        OSQP_num3 = OSQP_num3 + OSQP_num2;
        osqp.A_p_mpc[j + 1] = OSQP_num3;
        //        std::cout << "OSQP_A_p[" << j+1<< "]=" <<osqp.A_p[j + 1] << std::endl;
    }
    //        std::cout << "D" << std::endl;
    //////////////

    contact.Foot << 1, 1, 1, 1;

    if (contact.Foot(RL_FootIndex) == 0) {
        osqp.Fz_min(RL_FootIndex) = 0;
        osqp.Fz_max(RL_FootIndex) = 0;
    } else {
        osqp.Fz_min(RL_FootIndex) = 0;
        osqp.Fz_max(RL_FootIndex) = osqp.External_Fz_max;
    }

    if (contact.Foot(RR_FootIndex) == 0) {
        osqp.Fz_min(RR_FootIndex) = 0;
        osqp.Fz_max(RR_FootIndex) = 0;
    } else {
        osqp.Fz_min(RR_FootIndex) = 0;
        osqp.Fz_max(RR_FootIndex) = osqp.External_Fz_max;
    }

    if (contact.Foot(FL_FootIndex) == 0) {
        osqp.Fz_min(FL_FootIndex) = 0;
        osqp.Fz_max(FL_FootIndex) = 0;
    } else {
        osqp.Fz_min(FL_FootIndex) = 0;
        osqp.Fz_max(FL_FootIndex) = osqp.External_Fz_max;
    }

    if (contact.Foot(FR_FootIndex) == 0) {
        osqp.Fz_min(FR_FootIndex) = 0;
        osqp.Fz_max(FR_FootIndex) = 0;
    } else {
        osqp.Fz_min(FR_FootIndex) = 0;
        osqp.Fz_max(FR_FootIndex) = osqp.External_Fz_max;
    }

    /*
    osqp.Ref_GRF      <<  0, 0, 0, 0, 0, 0,
                          0, 0, 140,
                          0, 0, 140,
                          0, 0, 140,
                          0, 0, 140;
     */
    /*osqp.Ref_GRF      <<  0, 0, 0, 0, 0, 0,
                          0, 0, 1000,
                          0, 0, 1000,
                          0, 0, 1000,
                          0, 0, 1000;
     */

    //osqp.l  <<   -osqp.mu*abs(osqp.Ref_GRF(RL_Foot_AXIS_Z)), -osqp.mu*abs(osqp.Ref_GRF(RL_Foot_AXIS_Z)), osqp.Fz_min(RL_FootIndex), -osqp.mu*abs(osqp.Ref_GRF(RR_Foot_AXIS_Z)), -osqp.mu*abs(osqp.Ref_GRF(RR_Foot_AXIS_Z)), osqp.Fz_min(RR_FootIndex), -osqp.mu*abs(osqp.Ref_GRF(FL_Foot_AXIS_Z)), -osqp.mu*abs(osqp.Ref_GRF(FL_Foot_AXIS_Z)), osqp.Fz_min(FL_FootIndex), -osqp.mu*abs(osqp.Ref_GRF(FR_Foot_AXIS_Z)), -osqp.mu*abs(osqp.Ref_GRF(FR_Foot_AXIS_Z)), osqp.Fz_min(FR_FootIndex);
    //osqp.u  <<    osqp.mu*abs(osqp.Ref_GRF(RL_Foot_AXIS_Z)),  osqp.mu*abs(osqp.Ref_GRF(RL_Foot_AXIS_Z)), osqp.Fz_max(RL_FootIndex),  osqp.mu*abs(osqp.Ref_GRF(RR_Foot_AXIS_Z)),  osqp.mu*abs(osqp.Ref_GRF(RR_Foot_AXIS_Z)), osqp.Fz_max(RR_FootIndex),  osqp.mu*abs(osqp.Ref_GRF(FL_Foot_AXIS_Z)),  osqp.mu*abs(osqp.Ref_GRF(FL_Foot_AXIS_Z)), osqp.Fz_max(FL_FootIndex),  osqp.mu*abs(osqp.Ref_GRF(FR_Foot_AXIS_Z)),  osqp.mu*abs(osqp.Ref_GRF(FR_Foot_AXIS_Z)), osqp.Fz_max(FR_FootIndex);


    //WsJi

    /*
    for (unsigned int i = 0; i < osqp.data_A_nnz; ++i) {
        osqp.data_l[i] = osqp.l(i);
        osqp.data_u[i] = osqp.u(i);
    }*/
    // RL constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i] = -10000.;
        osqp.data_l_mpc[20 * i + 1] = -10000.;
        osqp.data_l_mpc[20 * i + 2] = 0.;
        osqp.data_l_mpc[20 * i + 3] = 0.;
        osqp.data_l_mpc[20 * i + 4] = 10.;
        osqp.data_u_mpc[20 * i] = 0.;
        osqp.data_u_mpc[20 * i + 1] = 0.;
        osqp.data_u_mpc[20 * i + 2] = 10000.;
        osqp.data_u_mpc[20 * i + 3] = 10000.;
        osqp.data_u_mpc[20 * i + 4] = 400.;
    }
    // RR constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 5] = -10000.;
        osqp.data_l_mpc[20 * i + 6] = -10000.;
        osqp.data_l_mpc[20 * i + 7] = 0.;
        osqp.data_l_mpc[20 * i + 8] = 0.;
        osqp.data_l_mpc[20 * i + 9] = 10.;
        osqp.data_u_mpc[20 * i + 5] = 0.;
        osqp.data_u_mpc[20 * i + 6] = 0.;
        osqp.data_u_mpc[20 * i + 7] = 10000.;
        osqp.data_u_mpc[20 * i + 8] = 10000.;
        osqp.data_u_mpc[20 * i + 9] = 400.;
    }
    // FL constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 10] = -10000.;
        osqp.data_l_mpc[20 * i + 11] = -10000.;
        osqp.data_l_mpc[20 * i + 12] = 0.;
        osqp.data_l_mpc[20 * i + 13] = 0.;
        osqp.data_l_mpc[20 * i + 14] = 10.;
        osqp.data_u_mpc[20 * i + 10] = 0.;
        osqp.data_u_mpc[20 * i + 11] = 0.;
        osqp.data_u_mpc[20 * i + 12] = 10000.;
        osqp.data_u_mpc[20 * i + 13] = 10000.;
        osqp.data_u_mpc[20 * i + 14] = 400.;
    }
    // FR constraint
    for (int i = 0; i < horizontal_length; ++i) {
        osqp.data_l_mpc[20 * i + 15] = -10000.;
        osqp.data_l_mpc[20 * i + 16] = -10000.;
        osqp.data_l_mpc[20 * i + 17] = 0.;
        osqp.data_l_mpc[20 * i + 18] = 0.;
        osqp.data_l_mpc[20 * i + 19] = 10.;
        osqp.data_u_mpc[20 * i + 15] = 0.;
        osqp.data_u_mpc[20 * i + 16] = 0.;
        osqp.data_u_mpc[20 * i + 17] = 10000.;
        osqp.data_u_mpc[20 * i + 18] = 10000.;
        osqp.data_u_mpc[20 * i + 19] = 400.;
    }
    /*
    double f_min = 0.;
    //double f_max = 200.;
    double f_max = 400.;


    osqp.data_l = {-10000., -10000., 0., 0., f_min,
                     -10000., -10000., 0., 0., f_min,
                     -10000., -10000., 0., 0., f_min,
                     -10000., -10000., 0., 0., f_min,                     
                    };

    osqp.data_u = {0., 0., 10000., 10000., f_max,
                     0., 0., 10000., 10000., f_max,
                     0., 0., 10000., 10000., f_max,
                     0., 0., 10000., 10000., f_max,
                    };
     */

    // Populate data


    if (OSQP_data) {
        OSQP_data->n = 12 * horizontal_length;
        OSQP_data->m = 20 * horizontal_length;
        OSQP_data->P = csc_matrix(OSQP_data->n, OSQP_data->n, osqp.P_nnz_mpc, osqp.P_x_mpc, osqp.P_i_mpc, osqp.P_p_mpc);
        OSQP_data->q = osqp.q_mpc_;
        OSQP_data->A = csc_matrix(OSQP_data->m, OSQP_data->n, osqp.A_nnz_mpc, osqp.A_x_mpc, osqp.A_i_mpc, osqp.A_p_mpc);
        OSQP_data->l = osqp.data_l_mpc;
        OSQP_data->u = osqp.data_u_mpc;
    }
    //    delete [] OSQP_A_x;
    //    free(OSQP_A_x);
    // Define solver settings as default
    if (OSQP_settings) {
        osqp_set_default_settings(OSQP_settings);
        OSQP_settings->alpha = 1; // Change alpha parameter
        OSQP_settings->verbose = false;
    }
    osqp_cleanup(OSQP_work);
    //    std::cout << "E" << std::endl;
    // Setup workspace
    osqp.exitflag = osqp_setup(&OSQP_work, OSQP_data, OSQP_settings);
    //    std::cout << "F" << std::endl;
    // Solve Problem
    osqp_solve(OSQP_work);
    //    if (OSQP_settings) c_free(OSQP_settings);
    cout << "[RL] x = " << OSQP_work->solution->x[0] << ", y = " << OSQP_work->solution->x[1] << ", z = " << OSQP_work->solution->x[2] << endl;
    cout << "[RR] x = " << OSQP_work->solution->x[3] << ", y = " << OSQP_work->solution->x[4] << ", z = " << OSQP_work->solution->x[5] << endl;
    cout << "[FL] x = " << OSQP_work->solution->x[6] << ", y = " << OSQP_work->solution->x[7] << ", z = " << OSQP_work->solution->x[8] << endl;
    cout << "[FR] x = " << OSQP_work->solution->x[9] << ", y = " << OSQP_work->solution->x[10] << ", z = " << OSQP_work->solution->x[11] << endl;

}

#endif