// struct Contact{
//     bool Con

// }


// struct Gain{
//     MatrixXf Kp_Joint, Kd_Joint, Kp_Cartesian, Kd_Cartesian;
// }
//// Enum
enum class GaitType {
  STAND,
//   STAND_CYCLE,
  STATIC_WALK,
//   AMBLE,
//   TROT_WALK,
  TROT,
//   TROT_RUN,
//   PACE,
//   BOUND,
//   ROTARY_GALLOP,
//   TRAVERSE_GALLOP,
//   PRONK,
//   THREE_FOOT,
//   CUSTOM,
//   TRANSITION_TO_STAND
};




////


struct EndPointFrame{
    EndPointState Ref;
    EndPointState Act;
}

struct BodyFrame{
    BaseState Ref;
    BaseState Act;
}

struct BaseState{
    // VectorXf Pos;
    // VectorXf Vel;
    // VectorXf Acc;
    Location LinearPos, LinearVel, LinearAcc;   // 변동될 수 있음
    Direction AngularPos, AngularVel, AngularAcc;

    MatrixXf R = Identity(3, 3);
    MatrixXf R_dot = Identity(3, 3);


    State{
        R = Identity(3, 3);
        R_dot = Identity(3, 3);
    }
}

struct EndPointState{
    Location LinearPos, LinearVel, LinearAcc;   // 변동될 수 있음

}

struct JointState{
    Direction AngularPos, AngularVel, AngularAcc;
    
    Torque torque;
}



struct Torque{
    float fric;
    float C;
    float G;

    float total;    
}


struct Location{
    Vector3f prev, now, goal;
}

struct Direction{
    Vector3f prev, now, goal;
}

struct EndPoint{
    EndPointFrame Global;
    EndPointFrame Base;

    // Gain gain;
    bool contact;

    VectorXf Kp(3);
    VectorXf Kd(3);
}

// EndPoint FL;
// FL.Global.Ref.LinearPos.now = ~~~


struct Joint{
    JointState Ref;
    JointState Act;
    // Torque RefTorque;
    // Torque ActTorque;

    float Kp;
    float Kd;
}



// joint[0].RefTorque = FL.gain.Kp_joint*(FL.Base.Ref.AngularPos - FL.Base.Act.AngularPos) + FL.gain.Kd_joint*(FL.Base.Ref.AngularVel - FL.Base.Act.AngularVel);

struct CoM{
    // EndPointFrame Global;
    // EndPointFrame Base;
    // EndPointFrame FootCoords;  
}


// 다리 좌표계는?

// CoM com;
// com.Global.Ref.LinearPos[2] = 

// class gait_data{
//     float t_period;
//     float t_swing;
        
// }


class OSQP{
public:
    OSQP(unsigned int horizontal_length);
    ~OSQP();
    float* P_x;


private:

}

OSQP::OSQP(unsigned int horizontal_length = 1)
{
    if(horizontal_length < 1){
        horizontal_length = 1;
    }

    P_x = new float[horizontal_length];
}

OSQP::~OSQP
{
    delete P_x;
}


class CRobot{
public:
    EndPoint FL, FR, RL, RR;
    Joint joint[12];
    CoM com;

    OSQP osqp(0);

    // 이 이후에 필요한 게 뭐가 있을까?
    // osqp osqp;
    // enum
    // 


    Trot trot;

    // Member function


};

