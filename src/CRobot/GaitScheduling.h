#ifndef _GAITSCHEDULING_
#define _GAITSCHEDULING_

class GaitSchedulingClass final
{
public:
  GaitSchedulingClass()
  {
    //   std::cout<<"<---Gait Init--->"<<std::endl;
  }
  ~GaitSchedulingClass()
  {
    //   std::cout<<"<---Gait End--->"<<std::endl;
  }

  void Gen_Traj(RobotClass &Robot);
  // ****************** Define Functions Below ********************//

  //***************************************************************//
};

void GaitSchedulingClass::Gen_Traj(RobotClass &Robot)
{
  Robot.Ref.EP.Pos << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
};

#endif
