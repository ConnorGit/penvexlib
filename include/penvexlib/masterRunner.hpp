/*
 * \file penvexlib/masterRunner.hpp
 *
 * BAD
 */

#ifndef _MASTER_HPP_
#define _MASTER_HPP_

#include "penvexlib/macros.hpp"
#include "penvexlib/mods/asyncLinearMotionProfileControllerMod.hpp"
#include "penvexlib/mods/asyncMeshMpPpController.hpp"

namespace penvex::master {

// TODO: This master file thing is terrible and needs to go but it is a
// nessicary evil at the moment

enum masterFunctionId { LD, RM, MP, PP, MPPP, TRN, DRV, STOP, WIS, WD, DTP };
enum masterFunctionSubsttId { B, I, C };
struct masterFunction {
  masterFunctionId funcId;
};
struct masterFunctionSubst : public masterFunction {
  masterFunctionSubsttId substId;
};
struct masterFunctionSubstPathId : public masterFunctionSubst {
  std::string pathId;
};
struct masterFunctionSubstPathIdRev : public masterFunctionSubstPathId {
  bool rev;
};
struct masterFunctionDouble : public masterFunction {
  double value;
};
struct masterFunctionDoubleXY : public masterFunctionDouble {
  double x;
  double y;
};

extern void runMasterFile(const std::string &idirectory,
                          const std::string &ipathId);

} // namespace penvex::master

#endif
