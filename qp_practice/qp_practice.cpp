/*
 * ============================================================
 * qp_practice Class
 * 
 * Robotics & Control Lab.
 * 
 * File                 : qp_practice.cpp
 * Author               : BK Cho
 *
 * First developer      : Jeong Hwan. Jang
 * Second developer     : 
 * 
 * Update date          : 2024. 06. 10 by Jeong Hwan. Jang
 * ============================================================
 */
                                 
#include "qp_practice/qp_practice.h"

qp_practice::qp_practice() {
}

qp_practice::~qp_practice() {
}

//* =============================================================================================================================================== *//
//* ============================================================ Quadratic Programming ============================================================ *//
//* =============================================================================================================================================== *//
/*
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * *** The process of finding the factor f(= f_desired) that minimizes the cost function.
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * 
 * f_desired = argmin [ (A * f - b).transpose() * S * (A * f - b) + alpha * f.transpose() * W * f  ]
 *
 * f : GRF (NUM_OF_LEG(6) * NUM_OF_POS(3) X 1 Vector) 
 *   
 *     _         [       f_1        ]
 *   _|_     =   [        :         ]  
 *    |          [   f_NUM_OF_LEG   ]
 * 
 * A : System State Matrix 
 *   # Skew Symmetric Matrix : M.transpose() = M.inverse() 
 *   # 6 X NUM_OF_LEG(6) * NUM_OF_POS(3) Matrix
 *  
 *     /\        [            I               ...                      I                 ]           
 *    /__\   =   [                                                                       ]  
 *   /    \      [  [CoM2Foot(Local),1 X]     ...     [CoM2Foot(Local),NUM_OF_LEG(6) X]  ]
 * 
 * b : System Reference 
 *   # NUM_OF_POS(3) + NUM_OF_ROT_Ang(3) X 1 Matrix
 * 
 *               [      m * (com.G_RefAcc + g)     ]
 *   |_      =   [                                 ]
 *   |_|         [   I_g * base.G_RefAngularAcc    ]
 * 
 * S : Positive-Definite Weight Matrix for Error 
 *   # 6 X 6 Matrix
 *   # Error : (A * f - b)
 *   # If the S weight is large, the tracking performance of CoM Pos and Ori improves.
 * 
 * W : Positive-Definite Weight Matrix for f
 *   # 3 * c X 3 * c Matrix (3 <= c <= 6)
 *   # 3 point support ~ 6 point support
 * 
 * C : Inequality Constraint Matrix
 *   # number of inequality constraint(5) X NUM_OF_LEG(6) * NUM_OF_POS(3) Matrix
 *   # GRFs lie inside the firiction cones & The normal components of the GRFs stay within some user-defined values.
 *   
 *    ____       [   (-mu_i * n_i + t_1i).transpose()   ]
 *   |           [   (-mu_i * n_i + t_2i).transpose()   ]
 *   |       =   [   ( mu_i * n_i + t_2i).transpose()   ]
 *   |____ i     [   ( mu_i * n_i + t_1i).transpose()   ]
 *               [          (n_i).transpose()           ]
 * 
 * 
 *    ____       [   C_0    ...    0    ] 
 *   |           [    :    .       :    ]
 *   |       =   [    :      .     :    ]
 *   |____       [    :        .   :    ]
 *               [    0     ...   C_c   ]
 * 
 * d : Constraint Lower & Upper Bound 
 * 
 *               [   - infinite   ]
 *               [   - infinite   ]
 *    _|     =   [       0        ]
 *   |_|         [       0        ]
 *   --- i       [     f_min_i    ]
 * 
 *               [       0        ]
 *   ---         [       0        ]
 *    _|     =   [    infinite    ]
 *   |_| i       [    infinite    ]
 *               [     f_max_i    ]
 * 
 * tau feedforward : Mapping the desired GRFs f_desired into joint space we get the feedforward torque.
 *   # tau_ff = - (S * J_c.transpose() * RotMat_Base2Global * f_desired)
 *              |                        ------------------
 *             \|/                                |__________________
 *       Why is there a negative definite?                           |
 *       --> Principle of action and reaction of GRF                \|/
 *                                                               Why multiply RotMat?
 *                                                               --> To view the obtained global GRF from a local perspective.
 * 
 * 
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * *** If you think of the above equation as an unknown number rather than a matrix and expand it, it is converted into a quadratic equation for f.
 * *** The problem can be solved using the converted expression using OSQP Solver.
 * *** Convert GRF(f_desired) obtained through OSQP Solver to (tau_desired).
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * 
 *     f_desired = argmin [ (A * f - b).transpose() * S * (A * f - b) + alpha * f.transpose() * W * f  ]
 *     
 *               = S * (A^2 * f^2 - 2 * A * b * f + b^2) + alpha * W * f^2
 *               
 *               = S * A^2 * f^2 + alpha * W * f^2 - 2 * S * A * b * f + + S * b^2
 *              
 *               = (1 / 2) * f.transpose() * 2 * (A.transpose() * S * A + alpha * W) * f + ((-2) * b.transpose() * S * A) * f
 *                                           ---------------------------------------       ------------------------------
 *                                                              |                                         |  
 *                                                             \|/                                       \|/                
 *                                                             ___                                                                       
 *                                                            |___|                                      ___                
 *                                                            |                                         |___|                           
 *                                                            |                                             |/
 *
 * 
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * *** Control of CoM's Position & Base's Orientation.
 * *** We Compute the desired acceleration(ddot_x_CoM_desired) of the CoM using PD Control.
 * *** We Compute the desired angular acceleration(dot_w_Base_desired) of the robot's Base 
 * *** -------------------------------------------------------------------------------------------------------------------------------------------- ***
 * 
 *   .. desired                 [      desired           current   ]                  [   .  desired    . current   ]
 *   \/           =   |/        [   \/           -    \/           ]   +   |/         [   \/           \/           ]
 *   /\ CoM           |\ p CoM  [   /\ CoM            /\ CoM       ]       |\ d CoM   [   /\ CoM       /\ CoM       ]
 *   
 * 
 *      .   desired                       ___        [    __ desired    __ T       ]                   [          desired              Current   ]
 *   \    /           =   |/         .   /___\   .   [   |__|          |__|        ]   +   |/          [   \    /           -   \    /           ]  
 *    \/\/  Base          |\ p Base      \___        [   |  \ Base     |  \ Base   ]       |\ d Base   [    \/\/  Base           \/\/  Base      ]
 *                                                      ------------  ------------                                               ---------------
 *                                                           |             |                                                            |
 *                                                          \|/           \|/                                                          \|/
 *                                              G_Base Reference Ori     G_Base Current Ori                                   G_Base_Current_AngularVel
 */                                                         
//* =============================================================================================================================================== *//
//* =============================================================================================================================================== *//
//* =============================================================================================================================================== *//

//* ======================================================================================================================================================================================================================== *//
//* ========================================================================================== Sparse Matrix Expression Algorithm ========================================================================================== *//
//* ======================================================================================================================================================================================================================== *//
/*
 * *** ------------------------------------------------------------------------------------------------------------ ***
 * *** ------------------------------ Description of Variables Used in the Algorithm ------------------------------ ***
 * *** ------------------------------------------------------------------------------------------------------------ ***
 * 
 * *** osqp.data_P_nnz[108] : P Matrix 의 상삼각 행렬에서 0이 아닌 원소의 총 개수 
 *                          * 최대 값 = P Matrix의 상삼각 행렬 원소의 총 개수
 *                          * Initial Value = 0
 * 
 * *** osqp.data_P_x[108]   : P Matrix 의 상삼각 행렬의 원소 중 0이 아닌 원소
 *                          * P_x 의 배열 수 = P_nnz 의 값
 *                          * 왼쪽 위 부터 한 열씩 체크
 * 
 * *** osqp.data_P_i[108]   : P Matrix 의 상삼각 행렬에서 0이 아닌 각각의 원소가 있는 행의 값
 *                          * 각각의 원소를 왼쪽 위 부터 한 열씩 체크할 때 원소의 행의 값
 * 
 * *** osqp.data_P_p[19]    : P Matrix 의 상삼각 행렬에서 그 열의 0이 아닌 원소의 누적 개수
 *                          * 첫 번째 배열은 0
 *                          * 마지막 배열은 누적 개수이므로 P_nnz의 값(P Matrix의 상 삼각 행렬에서 0이 아닌 원소의 총 개수)과 같음
 *                          * Initial Value = 0
 * 
 * *** temp_index           : 현재까지 처리된 P Matrix 의 상삼각 행렬의 0 이 아닌 원소 개수
 *                          * Initial Value = 0
 * 
 * *** --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ***
 * *** --------------------------------------------------------------------------------- Description of Sparse Matrix Expression Algorithm --------------------------------------------------------------------------------- ***
 * *** --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ***
 * 
 * for(int j = 0; j < P Matrix의 열의 개수[18]; ++j) --------------------------->     j : P Matrix의 열에 해당함 , 1열(즉, 왼쪽 위) 부터 한 열 씩 체크 
 * {
 *     for(int i = 0; i <= j; ++i) ------------------------------------------->     i : P Matrix의 행에 해당함
 *     {
 *         if(osqp.P(i, j) != 0) --------------------------------------------->     i,j 에 해당하는 P Matrix 의 상삼각 행렬 원소 값 osqp.P(i, j) 이 0 이 아닐 경우 실행 , 
 *         {                                                                        0 인경우 다음 i 값으로 넘어가며, osqp.data_P_x, osqp.data_P_i, osqp.data_P_p 에 저장되는 값이 없고, temp_index 의 값도 증가하지 않음 
 * 
 *             osqp.data_P_x[temp_index]    = osqp.P(i, j); ------------------>     i, j 에 해당하는 P Matrix 의 상삼각 행렬 원소 값 osqp.P(i, j) 을 osqp.data_P_x 배열에 저장
 *   
 *             osqp.data_P_i[temp_index]    = i; ----------------------------->     P Matrix 의 상삼각 행렬 원소의 행에 해당하는 i 를 osqp.data_P_i 배열에 저장
 *             
 *             osqp.data_P_nnz++; -------------------------------------------->     osqp.data_P_nnz 1 증가 >> P Matrix 의 상삼각 행렬에서 0이 아닌 원소의 개수를 셈
 *              
 *             if(!temp_flag) ------------------------------------------------>     temp_flag 가 false 일 때, j 열의 첫 번째 행에서만 실행                                                                     
 *             {                                                                                    
 *                 temp_flag                = true; -------------------------->     temp_flag 를 다시 true 로 변경 >> 즉, 다음 열(j)에 대한 계산이 시작되기 전 까지 다시 이 if 문에 들어오지 않음
 * 
 *                 osqp.data_P_p[j]         = temp_index; -------------------->     이전 열(j)까지 합산되고 있던 temp_index(현재까지 처리된 P Matrix 의 상삼각 행렬의 0 이 아닌 원소 개수)를 osqp.data_P_p 배열에 저장
 *             }
 *             ++temp_index; ------------------------------------------------->     현재까지 처리된 P Matrix 의 상삼각 행렬의 0 이 아닌 원소 개수 1 증가
 *         }
 *     }
 *     temp_flag = false; ---------------------------------------------------->     P Matrix 의 행에 대해 검토가 끝나면 temp_flag 를 false로 변경 >> 즉, 다음 열(j)로 넘어갈때 if 문 안으로 들어가기 위함
 * }
 * 
 * osqp.data_P_p[osqp.P.cols()] = temp_index; -------------------------------->     마지막 열[18]에서 현재까지 처리된 P Matrix 의 상삼각 행렬의 0 이 아닌 원소의 총 개수를 osqp.data_P_p 마지막 배열에 저장
 * 
 */
//* ======================================================================================================================================================================================================================== *//
//* ======================================================================================================================================================================================================================== *//
//* ======================================================================================================================================================================================================================== *//

void qp_practice::osqp_test()
{
    std::cout << "InitializeOSQP" << std::endl;
}

void qp_practice::InitializeOSQP(float Robot_Mass, Vector3f FL_G_CoM2Foot_Current, Vector3f FR_G_CoM2Foot_Current, Vector3f RL_G_CoM2Foot_Current, Vector3f RR_G_CoM2Foot_Current)
{
    //* ========== InitializeOSQP ========== *//
    /*
     * OSQP Initialize 하는 함수입니다.
     * 
     * 매개변수에는 로봇의 질량, CoM에서 각 발 끝까지의 현재 위치벡터가 필요합니다.
     * (이해를 돕기위해 매개변수를 넣었는데 함수에 꼭 입력이 필요하지 않다고 생각되면 꼭 매개변수를 사용하지 않으셔도 됩니다. 추가할게 있다면 하셔도 됩니다.)
     * 
     * 
     */

    // static int temp_index                               = 0;        //* 희소행렬을 만들 때 사용되는 변수입니다.
    // static bool temp_flag                               = false;    //* 희소행렬을 만들 때 사용되는 변수입니다.

    // osqp.I_g                                                     //* 로봇의 B_I_g 를 사용하여 정의
    
    // B_I_g                                           <<    1.339647,    -0.001957,    0.126605,
    //                                                      -0.001957,     3.848482,   -0.000377,
    //                                                       0.126605,    -0.000377,    4.377425;

    // CoM의 RefAcc                                     << 0., 0., 0.;
    // Base의 RefAngularAcc                             << 0., 0., 0.;

    // osqp.Mass                                        << Robot_Mass;

    // osqp.Gravity                                        << 0.,  0.,  GRAVITY;

    //* QP Gain Setting 
    //  CoM_QP_Kp                                            <<   80.,      0.,      0.,
    //                                                             0.,     80.,      0.,
    //                                                             0.,      0.,    100.; 

    //  CoM_QP_Kd                                           <<    2.,      0.,      0.,
    //                                                            0.,      2.,      0.,
    //                                                            0.,      0.,      3.; 

    //  BASE_QP_Kp                                          << 1500.,      0.,      0.,
    //                                                            0.,   1500.,      0., 
    //                                                            0.,      0.,    250.;

    //  BASE_QP_Kd                                          <<   60.,      0.,      0., 
    //                                                            0.,     60.,      0.,
    //                                                            0.,      0.,     25.;
    
    
    //* ========== A Matrix ========== *//
    // osqp.A.block<3, 3>(0, 0)                            = 
    // osqp.A.block<3, 3>(0, 3)                            = 
    // osqp.A.block<3, 3>(0, 6)                            = 
    // osqp.A.block<3, 3>(0, 9)                            = 
    
    // osqp.POS_CoM_CrossProduct                           <<

    // osqp.A.block<3, 18>(3, 0)                           =


    //* ========== b Matrix ========== *//
    // osqp.b                                              =

    //* ========== S Matrix ========== *// 
    // osqp.S(0, 0)                                        = 
    // osqp.S(1, 1)                                        = 
    // osqp.S(2, 2)                                        = 
    // osqp.S(3, 3)                                        =  
    // osqp.S(4, 4)                                        =  
    // osqp.S(5, 5)                                        =

    //* ========== W Matrix ========== *// 
    // for(size_t i = 0; i < NUM_OF_LEG; ++i)
    // {
    //     osqp.W(i * 3    , i * 3)                        = 1.0 * 15.0;         //* GRF_X
    //     osqp.W(i * 3 + 1, i * 3 + 1)                    = 1.0 * 15.0;         //* GRF_Y
    //     osqp.W(i * 3 + 2, i * 3 + 2)                    = 1.0 * 25.0;         //* GRF_Z
    // }

    //* ========== P Matrix ========== *//
    // osqp.P                                              = 
    
    //* ========== q Matrix ========== *//
    // osqp.q                                              =

    //* ========== Data Initialization for Sparse Matrix Expression ========== *//
    //* 희소행렬로 표현하기 전 변수 초기화
    // osqp.data_P_nnz                                     = 0;                                        
    // osqp.data_P_p[19]                                   = {0, };                    
                
    // temp_index                                          = 0;                 
    // temp_flag                                           = false;

    //* ====================== Sparse Matrix of P Matrix ====================== *//

    //* P Matrix를 희소행렬로 표현하는 과정 작성 ...

    //* ========== q Matrix data ========== *//
    // temp_index                                          = 0; //* 변수 초기화
    
    //* q Matrix를 osqp.data_q에 넣는 과정
    // for (unsigned int i = 0; i < osqp.q.rows(); ++i) 
    // {
    //     osqp.data_q[temp_index] = osqp.q(i);
    //     ++temp_index;
    // }


    // osqp.B_tangential_direction1                        << 1.000000, 0.000001, 0.000000;    //* Tangential Vector    (t_vector_1)
    // osqp.B_tangential_direction2                        <<-0.000001, 1.000000, 0.000000;    //* Tangential Vector    (t_vector_2)
    // osqp.B_SurfaceNormal                                << 0.000001, 0.000001, 1.000000;    //* SurfaceNormal Vector (n_vector)
    
    // osqp.G_tangential_direction1                        = osqp.B_tangential_direction1;
    // osqp.G_tangential_direction2                        = osqp.B_tangential_direction2;
    // osqp.G_SurfaceNormal                                = osqp.B_SurfaceNormal;

     //* ========== C Matrix ========== *//
    // osqp.C1_.block(0,0,1,3)                             = 
    // osqp.C1_.block(1,0,1,3)                             = 
    // osqp.C1_.block(2,0,1,3)                             = 
    // osqp.C1_.block(3,0,1,3)                             = 
    // osqp.C1_.block(4,0,1,3)                             = 

    // osqp.C2_                                            = 
    // osqp.C3_                                            = 
    // osqp.C4_                                            = 
        
    // osqp.C_.block(0,0,5,3)                              = 
    // osqp.C_.block(5,3,5,3)                              = 
    // osqp.C_.block(10,6,5,3)                             = 
    // osqp.C_.block(15,9,5,3)                             = 

    //* ===================== Sparse Matrix of C Matrix ====================== *//

    //* C_ Matrix를 희소행렬로 표현하는 과정 작성 ...

    //* ========== d Matrix - Set Upper & Lower Bound ========== *//
    // for(int i = 0; i < NUM_OF_LEG; ++i)
    // {
    //     osqp.data_l[5*i + 0]                            = 
    //     osqp.data_l[5*i + 1]                            = 
    //     osqp.data_l[5*i + 2]                            = 
    //     osqp.data_l[5*i + 3]                            = 
    //     osqp.data_l[5*i + 4]                            = 
                      
    //     osqp.data_u[5*i + 0]                            = 
    //     osqp.data_u[5*i + 1]                            = 
    //     osqp.data_u[5*i + 2]                            = 
    //     osqp.data_u[5*i + 3]                            = 
    //     osqp.data_u[5*i + 4]                            = 
    // }
    
    // osqp.data_m                                         = 
    // osqp.data_n                                         = 

    //* ========== Populate data ========== *//
    // if (OSQP_data)
    // {
    //     OSQP_data->n                                    = osqp.data_n;
    //     OSQP_data->m                                    = osqp.data_m;
    //     OSQP_data->P                                    = csc_matrix(OSQP_data->n, OSQP_data->n, osqp.data_P_nnz, osqp.data_P_x, osqp.data_P_i, osqp.data_P_p);
    //     OSQP_data->q                                    = osqp.data_q;
    //     OSQP_data->A                                    = csc_matrix(OSQP_data->m, OSQP_data->n, osqp.data_A_nnz, osqp.data_A_x, osqp.data_A_i, osqp.data_A_p);
    //     OSQP_data->l                                    = osqp.data_l;
    //     OSQP_data->u                                    = osqp.data_u;
    // }
    
    //* ========== Define solver settings as default ========== *//
    // if (OSQP_settings){
    //     osqp_set_default_settings(OSQP_settings);
    //     OSQP_settings->alpha                            = 1.0;          //* Change alpha parameter
    //     OSQP_settings->verbose                          = false;
    // }

    //* ========== Setup workspace ========== *//
    // osqp.exitflag                                       = osqp_setup(&OSQP_work, OSQP_data, OSQP_settings);
    
    //* ========== Solve Problem ========== *//
    // osqp_solve(OSQP_work);

    // for (unsigned int i = 0; i < NUM_OF_LEG * NUM_OF_POS; ++i)
    // {        
    //     osqp.Ref_GRF(i)                                 = OSQP_work->solution->x[i];   
    // }
}

void qp_practice::ProcessOSQP(Vector3f FL_G_CoM2Foot_Current, Vector3f FR_G_CoM2Foot_Current, Vector3f RL_G_CoM2Foot_Current, Vector3f RR_G_CoM2Foot_Current,
                              Vector3f Foot2CoM_RefPos, Vector3f Foot2CoM_CurrentPos, Vector3f Foot2CoM_RefVel, Vector3f Foot2CoM_CurrentVel,
                              Vector3f G_Base_RefOri, Vector3f G_Base_CurrentOri, Vector3f G_Base_RefAngularVel, Vector3f G_Base_CurrentAngularVel,
                              MatrixNd Jacobian_FL, MatrixNd Jacobian_FR, MatrixNd Jacobian_RL, MatrixNd Jacobian_RR)
{
    //* ========== ProcessOSQP ========== *//
    /*
     * OSQP Update 하는 함수입니다.
     * 
     * 매개변수에는 로봇의 질량, CoM에서 각 발 끝까지의 현재 위치벡터, CoM의 Ref, Current 위치 및 속도, Base의 Ref, Current Ori 및 AngularVel, 각 다리의 Jacobian이 필요합니다.
     * (이해를 돕기위해 매개변수를 넣었는데 함수에 꼭 입력이 필요하지 않다고 생각되면 꼭 매개변수를 사용하지 않으셔도 됩니다. 추가할게 있다면 하셔도 됩니다.)
     * 
     * 
     */

    //* ========== Update problem ========== *//
    // osqp_update_P(OSQP_work, osqp.data_P_x, OSQP_NULL, osqp.data_P_nnz);
    // osqp_update_A(OSQP_work, osqp.data_A_x, OSQP_NULL, osqp.data_A_nnz);
    // osqp_update_lin_cost(OSQP_work, osqp.data_q);
    // osqp_update_bounds(OSQP_work, osqp.data_l, osqp.data_u);

    //* ========== Solve Problem ========== *//
    // osqp_solve(OSQP_work);
    
    // for (unsigned int i = 0; i < NUM_OF_LEG * NUM_OF_POS; ++i) 
    // {        
    //     osqp.Ref_GRF(i)                                 = OSQP_work->solution->x[i];   
    // }

    // ROT_WORLD2BASE                              = EulerZYX2RotMat(base.G_CurrentOri);
    // ROT_BASE2WORLD                              = ROT_WORLD2BASE.transpose();

    // for (int i = 0; i < 4; i++) {
    //     ROT_WORLD2BASE_TOTAL.block(i*3, i*3, 3, 3) = ROT_WORLD2BASE;
    //     ROT_BASE2WORLD_TOTAL.block(i*3, i*3, 3, 3) = ROT_BASE2WORLD;
    // }

    // osqp.Jacobian.block( 0,0,3,18)              = J_FL.cast <float> ();
    // osqp.Jacobian.block( 3,0,3,18)              = J_FR.cast <float> ();
    // osqp.Jacobian.block( 6,0,3,18)              = J_RL.cast <float> ();
    // osqp.Jacobian.block( 9,0,3,18)              = J_RR.cast <float> ();

    // osqp.tmp_torque                             = -osqp.Select * osqp.Jacobian.transpose() * ROT_BASE2WORLD_TOTAL  * (osqp.Ref_GRF);

    // for (unsigned int i = 0; i < 12; ++i) {
    //     osqp.torque(i + BaseDOF)                = osqp.tmp_torque(i);
    // }
}