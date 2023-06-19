#include "MPCLocoMotion.h"
#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "convexMPC_interface.h"

MPCLocomotion::MPCLocomotion(double _dt, int _iterations_between_mpc):
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(5,5,5,5),"Galloping"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(6,6,6,6),"trotting"),
  standing(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10),"Standing"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(5,5,5,5),"Bounding"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  two_foot_trot(horizonLength, Vec4<int>(0,0,5,0), Vec4<int>(10,10,5,5), "two_foot_trot")
{
  dtMPC = dt * iterationsBetweenMPC;
  //std::cout << "dtMPC: " << dtMPC << std::endl;
  rpy_int[2] = 0;
  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");
}

void MPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData& data) 
{
  if((iterationCounter % 30) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    auto& stateCommand = data._desiredStateCommand;

    double* p = seResult.position.data();
    double* v = seResult.vWorld.data();
    double* w = seResult.omegaWorld.data();
    double* q = seResult.orientation.data();

    double r[12];
    for(int i = 0; i < 12; i++)
      r[i] = pFoot[i%4][i/4] - seResult.position[i/4];


    //double Q[12] = {15, 10, 5, 2.5, 2.5, 50, 0.0, 0.0, 0.3, 0.2, 0.2, 0.2}; // weight for aliengo
    double Q[12] = {15.0, 12.0, 10, 1.5, 1.5, 60, 0, 0, 0.3, 0.2, 0.2, 0.2};

    if(data._quadruped->robot_index == 2){
      *Q = (0.5, 0.5, 10, 2.5, 2.5, 20, 0, 0, 0.3, 0.4, 0.4, 0.4);
    } 
    double yaw = seResult.rpy[2];
    double* weights = Q;
    double alpha = 4e-5; // make setting eventually

    //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if(alpha > 1e-4)
    {

      std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
      alpha = 1e-5;
    }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    //Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;

      const double max_pos_error = .1;
      double xStart = world_position_desired[0];
      double yStart = world_position_desired[1];
   
      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error; 

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      //printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
      //printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);
      
      double trajInitial[12] = {//stateCommand->data.stateDes[3],
				rpy_comp[0] + stateCommand->data.stateDes[3],  // 0
                                rpy_comp[1] + stateCommand->data.stateDes[4],    // 1
                                stateCommand->data.stateDes[5],    // 2
                                xStart,                                   // 3
                                yStart,                                   // 4
                                0.3,   // 5
                                0,                                        // 6
                                0,                                        // 7
                                stateCommand->data.stateDes[11],  // 8
                                v_des_world[0],                           // 9
                                v_des_world[1],                           // 10
                                v_des_world[2]};                                       // 11
      if(data._quadruped->robot_index == 1)
      {
        trajInitial[5] = 0.4;
      }
      if(climb){
        trajInitial[1] += ground_pitch; 
      }

       for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
        if(i != 0)
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand->data.stateDes[11];
          //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }

    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC,horizonLength,0.4,550);
    update_solver_settings(1 /*_parameters->jcqp_max_iter*/,1e-7 /* _parameters->jcqp_rho */,
      1e-8 /*_parameters->jcqp_sigma*/, 1.5 /* _parameters->jcqp_alpha*/, 1 /*_parameters->jcqp_terminate*/,0 /* _parameters->use_jcqp*/);
    //t1.stopPrint("Setup MPC");
    //std::cout << t1.getMs() << std::endl;
    Timer t2;
    t2.start();
    //cout << "dtMPC: " << dtMPC << "\n";
    update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable, data._quadruped->robot_index);
    
    //t2.stopPrint("Run MPC");
    printf("MPC Solve time %f ms\n", t2.getMs());
    //std::cout << t2.getSeconds() << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
      Vec3<double> f;
      for(int axis = 0; axis < 3; axis++)
        f[axis] = get_solution(leg*3 + axis);
     
      //f_ff[leg] =  - coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * f;
      f_ff[leg] =  - seResult.rBody * f;
  
      //std::cout << "leg: " << leg << std::endl;
      //std::cout << f_ff[leg] << std::endl;
      //std::cout << "mpc solution" << leg << "\n" << seResult.rBody * f << std::endl;
      // Update for WBC
       // Fr_des[leg] = f;      
    }

    // printf("update time: %.3f\n", t1.getMs());
  }
}
