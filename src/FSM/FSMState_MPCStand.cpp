#include "../../include/FSM/FSMState_MPCStand.h"

FSMState_MPCStand::FSMState_MPCStand(ControlFSMData *data)
                 :FSMState(data, FSMStateName::MPCSTAND, "MPCStand")
{
    

}

void FSMState_MPCStand::enter()
{   
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // pipe.start(cfg);
    _data->_interface->zeroCmdPanel();
    _data->_legController->zeroCommand();
    _data->_legController->updateData(_data->_lowState, offset);
    _data->_stateEstimator->run();

    init_yaw = _data->_stateEstimator->getResult().rpy(2);

    for(int i = 0; i < 3; i++){
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }


    //set rpy command
    rpy[2] = init_yaw;

    //set COM command
    p_des[0] = 0;
    p_des[1] = 0;
    p_des[2] = 0.5;
}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}


void FSMState_MPCStand::run()
{
    motionTime++;

    std::cout << "Current state is MPC " << std::endl;

    pd = get_pose_data(pipe);

    std::cout << "Got data in pd!" << std::endl;
    // T265_x = pd.x;
    // T265_y = pd.y;
    // T265_z = pd.z;
    T265_pose[0] = pd.x;
    T265_pose[1] = pd.y;
    T265_pose[2] = pd.z;
    T265_pose[3] = pd.x_vel;
    T265_pose[4] = pd.y_vel;
    T265_pose[5] = pd.z_vel;
    
    T265_pos << -T265_pose[2] << "  " << -T265_pose[0] << " " << T265_pose[1] << "  " << -T265_pose[5] << " " << -T265_pose[3] << " " << T265_pose[4];

    joystick.pollEvents();
    state=joystick.getState();

    xAxis = state.axes[0];
    yAxis = state.axes[1];
    vx_command = -yAxis * 0.1;
    vy_command = -xAxis * 0.1;
    std::cout << "vx command is " << vx_command << std::endl;
    std::cout << "vy command is " << vy_command << std::endl;

    buttonA = state.buttons[0];
    buttonB = state.buttons[1];
    left_shoulder = state.buttons[4];

    if (left_shoulder) {
        abort();
    }

    if(motionTime > 800000){
        int rem = motionTime % 300;
        if( rem <= 150) {
        contactStateScheduled[0] = 1;
        contactStateScheduled[1] = 0;
        minForces[1] = 1;
        maxForces[1] = 2;
        }
        else {
        contactStateScheduled[0] = 0;
        contactStateScheduled[1] = 1;
        minForces[0] = 1;
        maxForces[0] = 2;
        }
        // std::cout << "motiontime rem: "<< rem << std::endl;
        // std::cout << "contact: "<< contactStateScheduled[0] << " , " << contactStateScheduled[1] << std::endl;
    }

    _data->_stateEstimator->setContactPhase(contactphase);

    v_des[0] = vx_command;
    v_des[1] = vy_command;
    v_command = {v_des[0], v_des[1], v_des[2]};

    // if (motiontime > 9000){
    //   v_des[0] =  0.1;
    // }

    // double target_time = 20000;
    // if (motiontime > target_time){
    //   // p_des[2] = 0.5 - (motiontime - target_time)/1000*0.05;
    //   // if (motiontime >= target_time+1000){
    //   //   p_des[2] = 0.45 + (motiontime - target_time+1000)/1000*0.05;
    //   // }
    //   // if (motiontime >= target_time+2000){
    //   //   p_des[2] = 0.5;
    //   // }

    //   p_des[2] = 0.03*sin(motiontime/1000.0 + 0.40)+0.47;
    // }


    // _data->_desiredStateCommand->convertToStateCommands();
    // std::cout << "test" << _data->_legController->data[0].q(0)*180/3.1415 << std::endl;
    // std::cout << "low state 1" << lowState.motorState[1].q  << std::endl;

    for (int i = 0; i < 12; i++){
        angle << _lowState->motorState[i].q << "  ";
    }


    _data->_legController->updateData(_data->_lowState, offset);

    if (!Calibration){
        //Angle Constraints
        if (motionTime > 5){
        // std::cout << _data->_legController->data[0].q(0)*180/3.1415 << std::endl;
        // Hip Constraint
        // if ((_data->_legController->data[0].q(0) < Abad_Leg1_Constraint[0]) || 
        //   (_data->_legController->data[0].q(0) > Abad_Leg1_Constraint[1])) {
        //     std::cout << "Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
        //     abort();
        //   }
        if ((_data->_legController->data[1].q(0) < Abad_Leg2_Constraint[0]) || 
            (_data->_legController->data[1].q(0) > Abad_Leg2_Constraint[1])) {
            std::cout << "Abad L Angle Exceeded" << _data->_legController->data[1].q(0) << std::endl;
            abort();
            }

        // AbAd Constraint
        if ((_data->_legController->data[0].q(1) < Hip_Leg1_Constraint[0]) ||
            (_data->_legController->data[0].q(1) > Hip_Leg1_Constraint[1])) {
            std::cout << "Hip R Angle Exceeded" << std::endl;
            abort();
            }
        if ((_data->_legController->data[1].q(1) < Hip_Leg2_Constraint[0]) ||
            (_data->_legController->data[1].q(1) > Hip_Leg2_Constraint[1])) {
            std::cout << "Hip L Angle Exceeded" << std::endl;
            abort();
            }

        //Thigh Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(2) < Thigh_Constraint[0]) || 
            (_data->_legController->data[leg].q(2) > Thigh_Constraint[1])) {
                std::cout << "Thigh Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Calf Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(3) < Calf_Constraint[0]) || 
            (_data->_legController->data[leg].q(3) > Calf_Constraint[1])) {
                std::cout << "Calf Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Ankle Constraint
        for (int leg = 0; leg < 2; leg++){
            if ((_data->_legController->data[leg].q(4) < Ankle_Constraint[0]) || 
            (_data->_legController->data[leg].q(4) > Ankle_Constraint[1])) {
                std::cout << "Ankle Angle Exceeded" << std::endl;
                abort();
            }
        }

        //Pitch Constraint
        if ((_data->_stateEstimator->getResult().rpy(1)) < -0.3){
            std::cout << "Pitch Angle Exceeded" << std::endl;
            abort();
        }
        }
    }

    _data->_legController->zeroCommand();

    _data->_stateEstimator->run();

    //std::cout << "Running QP" << std::endl;
    //////////////////// MPC ///////////////////
    if(motionTime >= 0){
        Cmpc.setGaitNum(7);

        _data->_desiredStateCommand->setStateCommands(roll, pitch, v_command, yaw_rate);
        Cmpc.run(*_data);

        std::cout << "MPC sol.:" << std::endl;
        for (int i = 0; i < 12; i++) {
          std::cout << get_solution(i) << std::endl;
        }

    //   Vec3<double> fOptR;
    //   Vec3<double> fOptO;
    //   Vec3<double> mOptR;
    //   Vec3<double> mOptO;

    //   for (int leg = 0; leg < 2; leg++){
    //   fOptO[0] = -get_solution(leg * 3);
    //   fOptO[1] = -get_solution(leg * 3 + 1);
    //   fOptO[2] = -get_solution(leg * 3 + 2);
    //   // fOptR = fOptO;
    //   // fOptR = _data->_stateEstimator->getResult().rBody * fOptO;
    //   fOptR = _data->_stateEstimator->getResult().rBody.transpose() * fOptO;
    //   mOptO[0] = -get_solution(6 + leg * 3);
    //   mOptO[1] = -get_solution(6 + leg * 3 + 1);
    //   mOptO[2] = -get_solution(6 + leg * 3 + 2);
    //   // mOptR = mOptO;
    //   mOptR = _data->_stateEstimator->getResult().rBody.transpose() * mOptO;
    //   // footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1], fOpt[leg * 3 + 2], fOpt[6 + leg * 3], fOpt[6 + leg * 3 + 1], fOpt[6 + leg * 3 + 2];
    //   footFeedForwardForces.col(leg) << fOptR[0], fOptR[1], fOptR[2], mOptR[0], mOptR[1], mOptR[2];
    //   // footFeedForwardForces.col(leg) << fOptO[0], fOptO[1], fOptO[2], mOptO[0], mOptO[1], mOptO[2];

    //   _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);

    // }

    }

    //Data Recording
    for (int leg = 0; leg < 2; leg++) {
        Vec3<double> pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose()*(_data->_quadruped->getHip2Location(leg) + _data->_legController->data[leg].p); //getResult().rBody *
            //Might need to delete that rBody later if the state estimation is not correct

        pFeet[leg * 3] = _data->_legController->data[leg].p[0];
        pFeet[leg * 3 + 1] = _data->_legController->data[leg].p[1];
        pFeet[leg * 3 + 2] = _data->_legController->data[leg].p[2];
        pFeetDes[leg * 3 ] = _data->_legController->commands[leg].pDes[0];
        pFeetDes[leg * 3 + 1] = _data->_legController->commands[leg].pDes[1];
        pFeetDes[leg * 3 + 2] = _data->_legController->commands[leg].pDes[2];
    }

    for (int i = 0; i <3; i++){
    com_pos << _data->_stateEstimator->getResult().position(i) << "  ";
    rpy_input << _data->_stateEstimator->getResult().rpy(i) << " ";
    }
    // for (int i = 0; i < 3; i++){
    //   com_pos << p_des[i] << "  ";
    // }
    for (int i = 0; i < 3; i++){
        com_pos << _data->_stateEstimator->getResult().vWorld(i) << "  ";
    }
    for (int i = 0; i < 3; i++){
        rpy_input << rpy[i] << "  ";
    }
    for (int i = 0; i < 13; i++){
        myfile << se_xfb[i] << "  ";
    }
    for (int i = 0; i <6; i++){
        footposition << pFeet[i] << "  ";
    }
    for (int i = 0; i <6; i++){
        footposition << pFeetDes[i] << "  ";
    }
    for (int i = 0; i < 12; i++){
        force << get_solution(i) << " ";
        // angle << lowState.motorState[i].q << "  ";
        QP << fOpt[i] << "  ";
    }

    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].q(i) << " ";
        }
    }
    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].qd(i) << " ";
        }
    }

    //Push the Command to Leg Controller
    _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);

    for (int i = 0; i < 12; i++){
        tau_est << _lowCmd->motorCmd[i].tau << "  ";
    }


    for (int i = 0; i < 12; i++){
        tau_est << _lowState->motorState[i].tauEst << "  ";
    }

    angle << std::endl;
    torque << std::endl;
    com_pos << std::endl;
    footposition << std::endl;
    QP << std::endl;
    myfile << std::endl;
    force << std::endl;
    rpy_input << std::endl;
    b_des << std::endl;
    tau_est << std::endl;
    corrected_angle<<std::endl;
    T265_pos << std::endl;

    // control.PowerProtect(_lowCmd, _lowState, 10); //TODO: This is commented out in the new framework; keep this way?
}

void FSMState_MPCStand::exit()
{
    _data->_interface->zeroCmdPanel();
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_MPCStand::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_X){ // TODO: Change this to the corrent UserCommand
        std::cout << "transition from MPC stand to QP stand" << std::endl;
        return FSMStateName::QPSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::MPCSTAND;
    }
}
