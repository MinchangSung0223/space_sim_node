#pragma once

#include <Eigen/Dense>
#include <stdexcept>

// 외부 Simulink 변수 선언
// extern real_T EQM_q_init_1, EQM_q_init_2, EQM_q_init_3;
// extern real_T EQM_q_init_4, EQM_q_init_5, EQM_q_init_6, EQM_q_init_7;
// extern real_T CRS_p_init_x, CRS_p_init_y, CRS_p_init_z;
// extern real_T TGT_p_init_x, TGT_p_init_y, TGT_p_init_z;
// extern real_T EQM_V1_FD_RPY_CRS[3];  // {yaw, pitch, roll}
// extern real_T EQM_V1_FD_RPY_TGT[3];

// extern ExtU_EQM_V1_FD_T EQM_V1_FD_U;
// extern ExtY_EQM_V1_FD_T EQM_V1_FD_Y;

class SpaceSim {
public:
    // 공개 멤버 (출력값들)
    Eigen::VectorXd q;            // CRS_q [7x1]
    Eigen::VectorXd qdot;         // CRS_qdot [7x1]
    Eigen::VectorXd CRS_SatPos;   // CRS_SatPos [7x1]
    Eigen::VectorXd CRS_SatVel;   // CRS_SatVel [6x1]
    Eigen::VectorXd TGT_SatPos;   // TGT_SatPos [7x1]
    Eigen::VectorXd TGT_SatVel;   // TGT_SatVel [6x1]
    Eigen::Vector2d gripper_q;   // gripper_q [2x1]
    

    SpaceSim() {
        q           = Eigen::VectorXd::Zero(7);
        qdot        = Eigen::VectorXd::Zero(7);
        CRS_SatPos  = Eigen::VectorXd::Zero(7);
        CRS_SatVel  = Eigen::VectorXd::Zero(6);
        TGT_SatPos  = Eigen::VectorXd::Zero(7);
        TGT_SatVel  = Eigen::VectorXd::Zero(6);
    }

    void initialize(const Eigen::VectorXd& EQM_q_init,
                    const Eigen::Matrix4d& CRS_T_init,
                    const Eigen::Matrix4d& TGT_T_init) ;

    void setInput(const Eigen::VectorXd& CRS_F0,
                  const Eigen::VectorXd& CRS_tau,
                  const Eigen::VectorXd& TGT_F0,
                  const Eigen::VectorXd& CRS_RW_tau,
                  const Eigen::VectorXd& TGT_RW_tau,
                  int gripper_state) ;
private:
    void assignRPY(const Eigen::Matrix4d& T, Eigen::Vector3d& rpy) {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d eul = R.eulerAngles(2, 1, 0);  // ZYX
        rpy[0] = eul[0];  // yaw
        rpy[1] = eul[1];  // pitch
        rpy[2] = eul[2];  // roll
    }
};
