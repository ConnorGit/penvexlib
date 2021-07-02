/**
 * \file scripts.hpp
 *
 * Contains the prototypes for auton scripts.
 */

#ifndef _SCRIPTS_HPP_
#define _SCRIPTS_HPP_

#include "penvexlib/core/macros.hpp"

namespace scripts {

// Full Autonomus Routines

extern void defaultAuton();
extern void defaultAutonInit();
extern void defaultAutonFreeDat();

extern void auton1();
extern void auton1Init();
extern void auton1FreeDat();

// Macros

extern penvex::Macro *macroTest;
extern void initMacroTest();

extern penvex::Macro *macroTest2;
extern void initMacroTest2();

} // namespace scripts
#endif // _SCRIPTS_HPP_
