/*
 * \file penvexlib/macros.hpp
 *
 * Contains prototypes and enums for macro functionality.
 */

#ifndef _MACROS_HPP_
#define _MACROS_HPP_

#include "api.h"
#include <vector>

namespace penvex::macro {

/**
 * Which subsystems the macro involves - to avoid competition and allow for
 * selective terminating should be defined in each subsystem with different
 * binary representaions. e.g. BASE = 0b0001 ARM = 0b0010
 */

/**
 * Struct used to store the data to run a macro.
 */
typedef struct {
  unsigned int usedSubsystems;
  void (*macroFunction)(void *);
  std::uint32_t prio = TASK_PRIORITY_DEFAULT;
  std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT;
  const char *name = "";
} macroData;

/**
 * This initalises the data neccary for to run macros, this could take a bit so run it in initalize.
 */
extern void init();

/**
 * Removes the currently running macros whose subsystems conflict with the
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
extern void runMacro(const macroData *macroToRun, void *macroFuncParams = NULL);

/**
 * Gets a list of the currently running macros.
 *
 * \Return a vector of the four currently running macros, if no macro exists in
 * a spot then it will return macroData{nullptr, 0b0000}
 */
extern std::vector<macroData> getRunningMacroData();

/**
 * Get all used subsystems as an ID musing the macro::subsystems enum.
 *
 * \Return bitwise or all cuttent used subsystems.
 */
extern unsigned int getAllUsedSubsystems();

} // namespace penvex::macro

#endif
