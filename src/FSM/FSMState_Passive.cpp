#include "../../include/FSM/FSMState_Passive.h"

FSMState_Passive::FSMState_Passive(ControlFSMData *data):
                  FSMState(data, FSMStateName::PASSIVE, "passive"){}

void FSMState_Passive::enter()
{
    _data->_legController->zeroCommand();

    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint << 5, 0, 0, 0, 0,
                                                      0, 5, 0, 0, 0,
                                                      0, 0, 5, 0, 0,
                                                      0, 0, 0, 5, 0,
                                                      0, 0, 0, 0, 5;
    }

}

void FSMState_Passive::run()
{
    std::cout << "Current state is passive state" << std::endl;
    _data->_legController->updateData(_data->_lowState, offset); //Pass by reference?
    _data->_stateEstimator->run();

    joystick.pollEvents();
    state=joystick.getState();
    xAxis = state.axes[0];
    std::cout << "Axes x is " << xAxis << std::endl;

    yAxis = state.axes[1];
    std::cout << "Axes y is " << yAxis << std::endl;

    // vx_command = -yAxis * 0.1;
    // vy_command = -xAxis * 0;
    // std::cout << "vx command is " << vx_command << std::endl;
    // std::cout << "vy command is " << vy_command << std::endl;

    buttonA = state.buttons[0];
    buttonB = state.buttons[1];
    left_shoulder = state.buttons[4];

    if (left_shoulder) {
        abort();
    }


    for (int i = 0; i < 12; i++){
    angle << _lowState->motorState[i].q << "  ";
    }

    for (int leg = 0; leg < 2; leg++)
    {
    for (int i = 0; i< 5; i++){
        corrected_angle << _data->_legController->data[leg].q(i) << " ";
    }
    }

    for (int leg = 0; leg < 2; leg++)
    {
    for (int i = 0; i< 5; i++){
        corrected_angle << _data->_legController->data[leg].qd(i) << " ";
    }
    }

    std::cout << "Orientation is" << std::endl;
    for (int i = 0; i < 4; i++) {
        // se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
        std::cout << _data->_stateEstimator->getResult().orientation(i) << std::endl;;
    }

    angle << std::endl;
    corrected_angle<<std::endl;

    _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);
}

void FSMState_Passive::exit()
{
    for(int i = 0; i < 2; i++)
    {
        _data->_legController->commands[i].kdJoint.setZero();
    }
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_Passive::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_X){
        std::cout << "transition from passive to QPstand" << std::endl;
        return FSMStateName::QPSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}