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

namespace testMacro {
extern penvex::Macro *macro;
extern void init();
extern void run();
extern void remove();
} // namespace testMacro

namespace test2Macro {
extern penvex::Macro *macro;
extern void init();
extern void run();
extern void remove();
} // namespace test2Macro

namespace balanceMacro {
extern penvex::Macro *macro;
extern void init();
extern void run();
extern void remove();
} // namespace balanceMacro

} // namespace scripts
#endif // _SCRIPTS_HPP_
