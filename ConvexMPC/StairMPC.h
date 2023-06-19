#ifndef STAIRMPC_H
#define STIARMPC_H

#include "MPCLocoMotion.h"

class StairMPC : public MPCLocomotion {
public:
  StairMPC(double _dt, int _iterations_between_mpc);

  void run(ControlFSMData& data);
};

#endif 