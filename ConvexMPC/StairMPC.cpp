#include <iostream>
#include "StairMPC.h"
#include "../include/common/Math/orientation_tools.h"

using namespace ori;

/* =========================== Controller ============================= */
StairMPC::StairMPC(double _dt, int _iterations_between_mpc):
 MPCLocomotion(_dt, _iterations_between_mpc)
{}

void StairMPC::run(ControlFSMData& data)
{
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  //std::cout << "in side mpc" << seResult.rBody << std::endl;;
  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &trotting;
  else if(gaitNumber == 3)
    gait = &walking;
  else if(gaitNumber == 4)
    gait = &pacing;
  else if(gaitNumber == 5)
    gait = &galloping;
  else if(gaitNumber == 6)
    gait = &pronking;
  else if(gaitNumber == 7)
    gait = &standing;
  else if(gaitNumber == 8)
    gait = &two_foot_trot;
    
  current_gait = gaitNumber;

  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + seResult.rBody.transpose() 
            * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
  }


  for(int i = 0; i < 4; i++)
  {
    W.row(i) << 1, footSwingTrajectories[i].getInitialPosition()[0], footSwingTrajectories[i].getInitialPosition()[1];
    pz[i] = footSwingTrajectories[i].getInitialPosition()[2];
         // if(i != 0) W.row(i) << 0, pFoot[i][0], pFoot[i][1];
    }
  a = W.transpose() * W * (W.transpose()* W * W.transpose()*W).inverse()*  W.transpose() * pz;
  ground_pitch = acos(-1/sqrt(a[1]*a[1] + a[2]*a[2] +1)) - 3.14;
      // std::cout << "ground pitch: " << ground_pitch << std::endl;
  if(pz[0] < pz[2]){
    ground_pitch = -ground_pitch;
  }
  if(abs(pz[0] - pz[2]) < 0.01){
    ground_pitch = 0;
  }

  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.3;
  
    // some first time initialization
  if(firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later
    iterationCounter = 0;
    Vec3<double> v_robot = seResult.vWorld;

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      //std::cout << "orig foot pos " << i << pFoot[i] << std::endl;
    }

    swingTimes[0] = dtMPC * gait->_swing;
    swingTimes[1] = dtMPC * gait->_swing;
    swingTimes[2] = dtMPC * gait->_swing;
    swingTimes[3] = dtMPC * gait->_swing;
    firstRun = false;
  }

 // printf("p_des \t%.6f\n", dt * v_des_world[0]);
 //Integral-esque pitch and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] - ground_pitch - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3] - ground_pitch - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking


  contact_state = gait->getContactSubPhase();
  double side_sign[4] = {-1, 1, -1, 1};
  double rear_sign[4] = {0, 0, -1, -1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
     if(v_des_robot[0] > 0.05)
        footSwingTrajectories[i].setHeight(.24);
     else
        footSwingTrajectories[i].setHeight(.1);

      Vec3<double> offset(rear_sign[i] * 0.0, side_sign[i] * 0.0838, 0);
      // simple heuristic function
      //std::cout << "swing time" << swingTimeRemaining[i] << std::endl;
      
      Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
      Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2) * pRobotFrame;
 
      Vec3<double> des_vel;
      
      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);

      Vec3<double> Pf = seResult.position +
                       seResult.rBody.transpose() * pYawCorrected
                       + seResult.vWorld* swingTimeRemaining[i];

      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.3;
      double pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[0]-v_des_world[0]) +
                      (0.5*seResult.position[2]/9.81) * (seResult.vWorld[1]*stateCommand->data.stateDes[11]);
      double pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[1]-v_des_world[1]) +
                      (0.5*seResult.position[2]/9.81) * (-seResult.vWorld[0]*stateCommand->data.stateDes[11]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] +=  pfx_rel;
      Pf[1] +=  pfy_rel;
      // + interleave_y[i] * interleave_gain * v_abs;
      Pf[2] = footSwingTrajectories[i].getInitialPosition()[2];

      footSwingTrajectories[i].setFinalPosition(Pf);  

  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  int* mpcTable = gait->mpc_gait();

   phase = gait->_phase;
  // load LCM leg swing gains
  // Kp << 400, 0, 0,
  //     0, 400, 0,
  //     0, 0, 200;
  // Kp_stance = 0*Kp;

  // Kd << 8, 0, 0,
  //     0, 8, 0,
  //     0, 0, 10;
  // Kd_stance = Kd;
  // gait
  Vec4<double> contactStates = gait->getContactSubPhase();
  Vec4<double> swingStates = gait->getSwingSubPhase();


  Vec3<double> vDesLeg_stand(0,0,0);

  for(int foot = 0; foot < 4; foot++)
  {
  
    double contactState = contactStates(foot);
    double swingState = swingStates(foot);

  
    //std::cout << "swing" << foot << ": " << swingState << std::endl;
    if(swingState > 0) // foot is in swing
    {
      
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);   

      }
      //std::cout << "swing" << foot << ": " << swingState << std::endl;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
        //  if(swingState > 0.5)
	//	        vDesLeg<< 0, 0, 0;
        data._legController->commands[foot].feedforwardForce << 0, 0, 0;
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpJoint.diagonal() << 60, 60, 60;
        data._legController->commands[foot].kdJoint.diagonal() << 1, 1, 1;
        // data._legController->commands[foot].kpCartesian = Kp;
        // data._legController->commands[foot].kdCartesian = Kd;

        // //account for early contact
        if(data._stateEstimator->getResult().contactEstimate(foot) != 0)
         {
           mpcTable[foot] = 1;
        //   // data._legController->commands[foot].feedforwardForce <<0, 0, -20;
         }
    }
    else if(contactState > 0 || data._stateEstimator->getResult().contactEstimate(foot) != 0) // foot is in stance
    {
      firstSwing[foot] = true;
       footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

        //  data._legController->commands[foot].kpCartesian = Kp_stance; // 0
        //  data._legController->commands[foot].kdCartesian = Kd_stance;
        data._legController->commands[foot].kpJoint.setZero();
        data._legController->commands[foot].kdJoint.diagonal() << 1,1, 1;
         data._legController->commands[foot].feedforwardForce = f_ff[foot];
    }
  }

  // estimate ground pitch


  updateMPCIfNeeded(mpcTable, data);  
  
  iterationCounter++;
}

