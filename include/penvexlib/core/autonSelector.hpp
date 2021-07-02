/**
 * \file penvexlib/autonSelector.hpp
 *
 * Contains prototypes for selecting between autonomus rutienes before match
 * using a macro.
 */

#ifndef _AUTON_SELECTOR_HPP_
#define _AUTON_SELECTOR_HPP_

#include "penvexlib/core/macros.hpp"
#include <iostream>
#include <vector>

namespace penvex {

// Autonomus names and pointers.
struct AutonData {
  std::string name;
  void (*autoFuncPtr)(void);
  void (*autoInitPtr)(void);
  void (*autoFreeDatPtr)(void);
};
extern const AutonData Autons[];
extern const unsigned char NUMBER_OF_AUTONS;

// For side selecting
typedef enum { BLUE = 0, RED } fieldSides;
// The side the robot starts on RED, of BLUE (all autos are written on blue)
extern fieldSides autonSide;

// The Autonomus function to call
extern void (*autonFunction)(void);
extern void (*autonInitFunction)(void);
extern void (*autonFreeDatFunction)(void);

namespace LCDSelector {
extern Macro *autonSelectorMacro;
extern void init();
} // namespace LCDSelector
} // namespace penvex

#endif // _AUTON_SELECTOR_HPP_
