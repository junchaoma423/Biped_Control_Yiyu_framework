#ifndef QPSTAND_H
#define QPSTAND_H

#include "FSMState.h"
#include "../../BalanceController/BalanceController.hpp"

class FSMState_QPStand: public FSMState
{
    public:
        FSMState_QPStand(ControlFSMData *data);
        ~FSMState_QPStand(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();

    private:
        BalanceController balanceController;
        int counter;
        Mat62<double> footFeedForwardForces; 
        Mat32<double> hiptoeforces;

        // Footstep locations for next step
        Mat34<double> footstepLocations;

        // QP Data
        double minForce = 1;
        double maxForce = 150; // change for different robot
        double minForces[2];
        double maxForces[2];
        double contactStateScheduled[2] = {1, 1}; //assume 2-leg standing
        double COM_weights_stance[3] = {1, 1, 1};
        double Base_weights_stance[3] = {1, 1, 1};
        double pFeet[6], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
               omegaDes[3];
        double se_xfb[13];
        double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
        double b_control[6];
        double init_yaw;

        Vec3<double> fOptR;
        Vec3<double> fOptO;
        Vec3<double> mOptR;
        Vec3<double> mOptO;

        // Saturation
        double _rollMax, _rollMin;
        double _pitchMax, _pitchMin;
        double _yawMax, _yawMin;
        double _heightMax, _heightMin;
        double _forwardMax, _backwardMax;

};

#endif