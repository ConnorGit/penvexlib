/*
 * \file penvexlib/macros.cpp
 *
 * Provides macro functionality.
 */

#include "main.h"
#include <algorithm>

namespace penvex::macro {

/**
 * Should be invoked whenever running macros is being written to.
 */
pros::Mutex runningMacroMutex;

/**
 * A list containing every currently running macro and its data.
 */
macroData *runningMacros[numberOfSubsystemsBuffer] = {nullptr, nullptr, nullptr,
                                                      nullptr, nullptr};

void breakMacros(unsigned int subsystemsToBreak) {
  runningMacroMutex.take(TIMEOUT_MAX);

  // Bitwise & used as a mask for usedSubsustems
  for (int i = 0; i < numberOfSubsystems; i++)
    if ((runningMacros[i] != nullptr) &&
        (runningMacros[i]->usedSubsystems & subsystemsToBreak)) {

      runningMacros[i]->macroTask->suspend();
      printf("removing one/n");

      // NOTRE: WARNING MAKE SURE NOT TO LEAK MEMORY ON REMOVED TASKS
      if (runningMacros[i]->restartOnBreak)
        runningMacros[i]->macroTask->remove();

      runningMacros[i] = nullptr;
    }

  runningMacroMutex.give();
}

void runMacro(macroData *macroToRun, void *macroFuncParams) {
  // You cant run a macro with no subsystems
  if (macroToRun->usedSubsystems == 0b0) {
    printf("Failed to run macro with no subsystems.");
    return;
  }

  // End all macros that conflict in subsystem usage
  breakMacros(macroToRun->usedSubsystems);
  // Run the macro
  if ((macroToRun->macroTask != nullptr) &&
      macroToRun->macroTask->get_state() == pros::E_TASK_STATE_SUSPENDED)
    macroToRun->macroTask->resume();
  else
    macroToRun->macroTask = new pros::Task(
        macroToRun->macroFunction, macroFuncParams, macroToRun->prio,
        macroToRun->stack_depth, macroToRun->name);

  // Add pointer to the the started macro tho the list of running macros
  runningMacroMutex.take(TIMEOUT_MAX);
  for (int i = 0; i < numberOfSubsystems; i++)
    if (runningMacros[i] == nullptr) {
      runningMacros[i] = macroToRun;
      break;
    }
  runningMacroMutex.give();
}

void initMacro(macroData *macroToRun, void *macroFuncParams) {

  std::uint32_t tempPrio = macroToRun->prio;
  macroToRun->prio = TASK_PRIORITY_MIN;

  runMacro(macroToRun, macroFuncParams);
  macroToRun->macroTask->suspend();

  macroToRun->prio = tempPrio;
}

macroData **getRunningMacroData(macroData *fillArr[]) {
  for (int i = 0; i < numberOfSubsystems; i++)
    fillArr[i] = runningMacros[i];
  return fillArr;
}

unsigned int getAllUsedSubsystems() {
  unsigned int allUsedSubsystem = 0b0;
  for (int i = 0; i < numberOfSubsystems; i++)
    if (runningMacros[i] != nullptr)
      allUsedSubsystem |= runningMacros[i]->usedSubsystems;
  return allUsedSubsystem;
}

} // namespace penvex::macro
