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
 * Contains the data of a currently running macro most usefully the Id and a
 * pointer to the task.
 */
struct runningMacro {
  macroData data; // This stores the subsystem ids and function pointers for the
                  // currently stored macro in the task
  pros::Task
      *macroTask; // This is one of the four tasks initalized to run macros
};

/**
 * A list containing every currently running macro and its data.
 */
std::vector<runningMacro> runningMacros;

void breakMacros(unsigned int subsystemsToBreak) {
  runningMacroMutex.take(TIMEOUT_MAX);

  // Bitwise & used as a mask for usedSubsustems
  runningMacros.erase(std::remove_if(
      runningMacros.begin(), runningMacros.end(),
      [subsystemsToBreak](runningMacro x) {
        x.macroTask->remove(); // TODO: Does remove leek data, I would think
                               // that it wouldnt but should prob test?
        return x.data.usedSubsystems & subsystemsToBreak;
      }));

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
  runningMacro newMacro;
  newMacro.data = *macroToRun;
  newMacro.macroTask = new pros::Task(
      newMacro.data.macroFunction, macroFuncParams, newMacro.data.prio,
      newMacro.data.stack_depth, newMacro.data.name);
  runningMacroMutex.take(TIMEOUT_MAX);
  runningMacros.emplace_back(newMacro);
  runningMacroMutex.give();
}

std::vector<macroData> getRunningMacroData() {
  std::vector<macroData> dataReturn;
  for (std::vector<runningMacro>::iterator vectorItter = runningMacros.begin();
       vectorItter != runningMacros.end(); vectorItter++)
    dataReturn.emplace_back(vectorItter->data);
  return dataReturn;
}

unsigned int getAllUsedSubsystems() {
  unsigned int allUsedSubsystem = 0b0000;
  for (std::vector<runningMacro>::iterator vectorItter = runningMacros.begin();
       vectorItter != runningMacros.end(); vectorItter++)
    allUsedSubsystem |= (*vectorItter).data.usedSubsystems;
  return allUsedSubsystem;
}

} // namespace penvex::macro
