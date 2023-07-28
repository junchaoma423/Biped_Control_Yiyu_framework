#include "../../include/FSM/FSMState.h"

float FSMState::T265_pose[6] = {0, 0, 0, 0, 0, 0};
bool FSMState::rs2_initialized = false;
rs2::pipeline FSMState::pipe;

FSMState::FSMState(ControlFSMData *data, FSMStateName stateName, std::string stateNameStr):
            _data(data), _stateName(stateName), _stateNameStr(stateNameStr), control(UNITREE_LEGGED_SDK::LeggedType::A1), Cmpc(0.001,30)
{
    _lowCmd = _data->_lowCmd;
    _lowState = _data->_lowState;

    // From old biped controller
    std::cout << "init FSM" << std::endl;
    
    myfile.open("ori.txt");
    QP.open("QPsolution.txt");
    com_pos.open("COMpos.txt");
    b_des.open("b_des_z.txt");
    angle.open("angle.txt");
    torque.open("torque.txt");
    footposition.open("foot.txt");
    rpy_input.open("rpy_files.txt");
    force.open("force.txt");
    omega.open("omega.txt");
    acceleration.open("acceleration.txt");
    tau_est.open("Estimated_torque.txt");
    corrected_angle.open("corrected_angle.txt");
    T265_pos.open("T265_pos.txt");

    offset = Angle_Caliberation();

    if (!rs2_initialized) {
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        pipe.start(cfg);
        rs2_initialized = true;
    }

    // float T265_pose[6] = {0.0f};
}

// double* FSMState::Angle_Caliberation() {
//     std::ifstream angle_file;
//     std::string angle_name;
//     int i = 0;

//     angle_file.open("../Calibration/offset.txt");
//     // angle_file.open("/home/junchao/Desktop/OldToNewFramework/Biped_Control_Yiyu_framework-main/Calibration/offset.txt"); //Fix this

//     // Check if the file opened successfully
//     if (!angle_file.is_open()) {
//         std::cerr << "Error opening offset.txt file!" << std::endl;
//         std::exit(1); // or return an error value
//     }

//     getline(angle_file, angle_name);
//     std::cout << "Angle string is " << angle_name << std::endl;

//     std::stringstream ss(angle_name);
//     double angle1, angle2, angle3, angle4, angle5, 
//            angle6, angle7, angle8, angle9, angle10;
    
//     ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10;

//     // // Check if all angles were read successfully
//     // if (!ss) {
//     //     std::cerr << "Error reading all angles from the stringstream." << std::endl;
//     //     exit(1); // or return an error value
//     // }

//     // Verify read values
//     std::cout << "Read angles: " << angle1 << " " << angle2 << " " << angle3 << " " 
//               << angle4 << " " << angle5 << " " << angle6 << " " << angle7 << " "
//               << angle8 << " " << angle9 << " " << angle10 << std::endl;

//     static double offset_angle[10];
    
//     offset_angle[0] = angle1;
//     offset_angle[1] = angle2;
//     offset_angle[2] = angle3;
//     offset_angle[3] = angle4;
//     offset_angle[4] = angle5;
//     offset_angle[5] = angle6;
//     offset_angle[6] = angle7;
//     offset_angle[7] = angle8;
//     offset_angle[8] = angle9;
//     offset_angle[9] = angle10;

//     std::cout << "Offset in function is " << angle1 << std::endl;

//     return offset_angle;
// }


double *FSMState::Angle_Caliberation(){
    std::ifstream angle_file;
    std::string angle_name;
    int i = 0;

    angle_file.open("../Calibration/offset.txt");

    getline(angle_file, angle_name);
    std::cout << "Angle string is " << angle_name << std::endl;

    std::stringstream ss(angle_name);
    double angle1, angle2, angle3, angle4, angle5, 
            angle6, angle7, angle8, angle9, angle10;
    ss >> angle1 >> angle2 >> angle3 >> angle4 >> angle5 >> angle6 >> angle7 >> angle8 >> angle9 >> angle10;

    static double offset_angle[10] = {angle1, angle2, angle3, angle4, angle5, angle6, angle7, angle8, angle9, angle10};
    std::cout << "Offset in function is " << angle1 << std::endl;

    return offset_angle;
}