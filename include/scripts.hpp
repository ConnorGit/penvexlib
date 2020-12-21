/**
 * \file scripts.hpp
 *
 * Contains the prototypes for auton scripts.
 */

#ifndef _SCRIPTS_HPP_
#define _SCRIPTS_HPP_

#include "penvexlib/macros.hpp"

namespace scripts {

// Macros

extern penvex::macro::macroData macroTest_data;
extern void runMacroTest();
extern void initMacroTest();

extern penvex::macro::macroData macroTest2_data;
extern void runMacroTest2();
extern void initMacroTest2();

} // namespace scripts
#endif // _SCRIPTS_HPP_
