#ifndef NOMINALMPC_H
#define NOMINALMPC_H

#include "MPCLocoMotion.h"

class NominalMPC : public MPCLocomotion {
public:
  NominalMPC(double _dt, int _iterations_between_mpc);
  void run(ControlFSMData& data);
};


#endif 