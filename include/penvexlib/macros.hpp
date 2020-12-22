/*
 * \file penvexlib/macros.hpp
 *
 * Contains prototypes and enums for macro functionality.
 */

#ifndef _MACROS_HPP_
#define _MACROS_HPP_

#include "api.h"

namespace penvex::macro {

/**
 * The count of subsystems the macro involves - to avoid competition and allow
 * for selective terminating should be defined in each subsystem with different
 * binary representaions. e.g. BASE = 0b0001 ARM = 0b0010
 */
extern const unsigned int numberOfSubsystems;
const unsigned int numberOfSubsystemsBuffer =
    5; // NOTE: Must be greator than or equal to numberOfSubsystems - and if you
       // change this then also change the array init in macros.cpp

/**
 * Struct used to store the data to run a macro.
 */
typedef struct {
  unsigned int usedSubsystems;
  void (*macroFunction)(void *);
  bool restartOnBreak;
  std::uint32_t prio = TASK_PRIORITY_DEFAULT;
  std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT;
  const char *name = "";
  pros::Task *macroTask = nullptr; // Do not init this manualy
} macroData;

/**
 * Stops the currently running macros whose subsystems conflict with the
subsystems given.
 *
 * \Param subsystemsToBreak The ids of the subsystems that if a currently
 * running macro matches will cause the running macro to terminate.
 */
extern void breakMacros(unsigned int subsystemsToBreak);

/**
 * Run a macro via its macro data.
 * This should be only called via a deducated runMacroName() function within a
 * macro file.
 *
 * \Param macroToRun pointer to the macroData of the macro to run.
 */
extern void runMacro(macroData *macroToRun, void *macroFuncParams = NULL);

/**
 * This initalises task of a macro than stops it - this should be called for
 * every macro in init
 */
extern void initMacro(macroData *macroToRun, void *macroFuncParams = NULL);

/**
 * This should only be called at the end of a restart macro to remove it form the list of running macros.
 */
extern void endMacro(unsigned int subsystemsToBreak);

/**
 * Fills an array of size numberOfSubsystems and type macroData with pointers to
 * the currently running macros.
 *
 * \Param arr pointer to the array to fill.
 * \Return a pointer to the filed array.
 */
extern macroData **getRunningMacroData(macroData *fillArr[]);

/**
 * Get all used subsystems as an ID musing the macro::subsystems enum.
 *
 * \Return bitwise or all cuttent used subsystems.
 */
extern unsigned int getAllUsedSubsystems();

} // namespace penvex::macro

#endif
