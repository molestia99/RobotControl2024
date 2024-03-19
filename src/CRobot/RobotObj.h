#ifndef _ROBOTOBJ_
#define _ROBOTOBJ_

enum class MODE_TYPE
{
  NONE = 0,
  REMOTE_WALK,
  REMOTE_DRIVE,
  WAYPOINT,
  FOLLOWING,
  TRACTION,
  RECOVERY
};

enum class FLAGSTATE
{
  NONE = 0,
  DECELERATE,
  CHANGE_POSTURE,
  ACTIVATE_MODE
};

static const char *enum_string_modestate[] = {"NONE", "DECELERATE", "CHANGE_POSTURE", "ACTIVATE_MODE"};

struct FLAGS
{
  bool RecoveryJudge;
  bool FootholdPoint;
  bool RecoveryGen;
  bool GaitTraj;
  bool HoldTraj;
  bool WalkReadyTraj;
  bool DriveReadyTraj;
  bool TorqueGen;
  bool WalkController;
  bool ActiveDriveController;
  bool PassiveDriveController;
  bool WayPointGen;
  bool FollowingGen;
};

struct MODE
{
  MODE_TYPE Now;
  MODE_TYPE Ref;
  FLAGSTATE State;
  FLAGS Flag;
};

static const char *enum_string_mode[] = {"NONE", "REMOTE_WALK", "REMOTE_DRIVE", "WAYPOINT", "FOLLOWING", "TRACTION", "RECOVERY"};

enum class ROBOTSTATE
{
  NONE = 0,
  STOP,
  MOVE,
  FALL
};
static const char *enum_string_state[] = {"NONE", "STOP", "MOVE", "FALL"};

enum Keyboard
{
  UP = 1073741906,
  DOWN = 1073741905,
  LEFT = 1073741904,
  RIGHT = 1073741903,
  W = 119,
  S = 115,
  A = 97,
  D = 100,
  Q = 113,
  E = 101,
  R = 114,
  Num1 = 49,
  Num2 = 50,
  Num3 = 51,
  Num4 = 52,
  Num5 = 53,
  Num6 = 54,
  Num7 = 55,
  Num8 = 56,
  Num9 = 57
};

struct JOINT{
  Eigen::VectorXd init_Pos;
  Eigen::VectorXd goal_Pos;
  Eigen::VectorXd Pos;
  Eigen::VectorXd Vel;
  Eigen::VectorXd Torque;
};
struct ENDPOINT{
  Eigen::VectorXd Foothold;
  Eigen::VectorXd Pos;
  Eigen::VectorXd Vel;
  Eigen::VectorXd preVel;
};
struct BODY{
  Eigen::VectorXd Pos; 
  Eigen::VectorXd Vel;
  Eigen::VectorXd preVel;
  Eigen::VectorXd Acc;
  Eigen::VectorXd AngVel;
  Eigen::VectorXd Ori;
  Eigen::MatrixXd R;
};
struct WHEEl{
  Eigen::VectorXd Pos;
  Eigen::VectorXd Vel; 
  Eigen::VectorXd Torque;
  double Brake;
};
struct CYLINDER{
  double Pos;
  double Vel;
  double Force;
};

struct BATTERY{
  double Cur;
};

struct REFERENCE
{
  JOINT Joint;
  ENDPOINT EP;
  BODY Body;
  WHEEl Wheel;
  CYLINDER Cylinder;
  BATTERY Bat;
  Eigen::MatrixXd J;
  // Eigen::VectorXd ActiveDriveCommand;
  // Eigen::VectorXd PassiveDriveCommand;
};

struct ACTUAL
{
  JOINT Joint;
  ENDPOINT EP;
  BODY Body;
  WHEEl Wheel;
  CYLINDER Cylinder;
  BATTERY Bat;
  Eigen::MatrixXd J;
};

struct GAIN
{
  Eigen::VectorXd JointPgain;
  Eigen::VectorXd JointDgain;
};
struct MAP{
  Eigen::VectorXd Height;
  Eigen::VectorXd Slope;
};

struct RBDL
{
  int ID_LF;
  int ID_RF;
  int ID_LH;
  int ID_RH;
};

struct OCU
{
  Eigen::VectorXd MaxBodyVel;
  Eigen::VectorXd MaxBodyAngVel;
  Eigen::VectorXd ButtonON;
  bool test;
};

class RobotClass final
{
private:
  std::vector<int> SensorData;

public:
  REFERENCE Ref;
  ACTUAL Act;
  MODE Mode;
  ROBOTSTATE RobotState;
  GAIN Gain;
  RBDL Rbdl;
  OCU Ocu;
  MAP Map;
  bool Done;
  int cnt;
  const double StopThreshold = 0.01;

  RobotClass()
  {
    std::cout << "<---Robot Init--->" << std::endl;
    Mode.Now = MODE_TYPE::NONE; //{NONE,REMOTE_WALK,REMOTE_DRIVE,WAYPOINT,FOLLOWING,TRACTION,RECOVERY}
    Mode.Ref = MODE_TYPE::NONE;
    Mode.State= FLAGSTATE::NONE;

    RobotState = ROBOTSTATE::STOP; // ROBOTSTATE::NONE, STOP, MOVE, FALL

    Done = false;
    Mode.Flag.RecoveryJudge = true; // Always ON
    Mode.Flag.FootholdPoint = false;
    Mode.Flag.RecoveryGen = false;
    Mode.Flag.GaitTraj = false;
    Mode.Flag.HoldTraj = false;
    Mode.Flag.DriveReadyTraj = false;
    Mode.Flag.WalkReadyTraj = false;
    Mode.Flag.TorqueGen = false;
    Mode.Flag.WalkController = true;
    Mode.Flag.ActiveDriveController = false;
    Mode.Flag.PassiveDriveController = false;
    Mode.Flag.WayPointGen = false;
    Mode.Flag.FollowingGen = false;
    cnt = 0;

    Ref.Joint.Torque = Eigen::VectorXd::Zero(12);
    Ref.Joint.init_Pos = Eigen::VectorXd::Zero(12);
    Ref.Joint.goal_Pos = Eigen::VectorXd::Zero(12);
    Ref.Joint.Pos = Eigen::VectorXd::Zero(12);
    Ref.Joint.Vel = Eigen::VectorXd::Zero(12);
    Ref.EP.Foothold = Eigen::VectorXd::Zero(12);
    Ref.EP.Pos = Eigen::VectorXd::Zero(12);
    Ref.EP.Vel = Eigen::VectorXd::Zero(12);

    Ref.Body.Vel = Eigen::VectorXd::Zero(3);
    Ref.Body.AngVel = Eigen::VectorXd::Zero(3);
    Ref.Wheel.Torque = Eigen::VectorXd::Zero(2);
    Ref.Cylinder.Pos = 0;
    Ref.Cylinder.Force = 0;
    Ref.Wheel.Brake = 0;
    Ref.Bat.Cur = 0;

    Act.Joint.Pos = Eigen::VectorXd::Zero(12);
    Act.Joint.Vel = Eigen::VectorXd::Zero(12);
    Act.EP.Pos = Eigen::VectorXd::Zero(12);
    Act.EP.Vel = Eigen::VectorXd::Zero(12);
    Act.Body.Vel = Eigen::VectorXd::Zero(3);
    Act.Body.preVel = Eigen::VectorXd::Zero(3);
    Act.Body.Acc = Eigen::VectorXd::Zero(3);
    Act.Body.AngVel = Eigen::VectorXd::Zero(3);
    Act.Body.Ori = Eigen::VectorXd::Zero(3);
    Act.Wheel.Pos = Eigen::VectorXd::Zero(2);
    Act.Cylinder.Pos = 0;
    Act.Body.R=Eigen::MatrixXd::Identity(3,3);
    Act.J = Eigen::MatrixXd::Zero(12, 12);
    Act.Wheel.Brake = 0;
    Act.Bat.Cur = 0;

    Gain.JointPgain = Eigen::VectorXd::Zero(12);
    Gain.JointDgain = Eigen::VectorXd::Zero(12);
    Gain.JointPgain.setConstant(2500.0);
    Gain.JointDgain.setConstant(100.0);
  
    Ocu.MaxBodyVel = Eigen::VectorXd::Zero(3);
    Ocu.MaxBodyAngVel = Eigen::VectorXd::Zero(3);
    Ocu.test = false;

    Map.Height=Eigen::VectorXd::Zero(3);
    Map.Slope=Eigen::VectorXd::Zero(2);
  }
  ~RobotClass()
  {
    std::cout << "<---Robot end--->" << std::endl;
  }
  bool isDriveModeSet(enum MODE_TYPE &mode);
  bool isRobotStop(void);
};

bool RobotClass::isDriveModeSet(enum MODE_TYPE &mode)
{
  //   std::cout<<"Mode is "<<mode<<std::endl;
  return (mode == MODE_TYPE::REMOTE_DRIVE || mode == MODE_TYPE::WAYPOINT || mode == MODE_TYPE::FOLLOWING || mode == MODE_TYPE::TRACTION);
};

#endif
