/*
 * \file penvexlib/macros.cpp
 *
 * Provides macro functionality.
 */

#include "main.h"
#include <algorithm>

namespace penvex::macro {

struct runningMacro {
  macroData data; // This stores the subsystem ids and function pointers for the
                  // currently stored macro in the task
  pros::Task
      *macroTask; // This is one of the four tasks initalized to run macros
};

std::vector<runningMacro> runningMacros;

void breakMacros(unsigned int subsystemsToBreak) {
  // Bitwise & used as a mask for usedSubsustems
  runningMacros.erase(
      std::remove_if(runningMacros.begin(), runningMacros.end(),
                     [subsystemsToBreak](runningMacro x) {
                       x.macroTask->remove(); // TODO: Does remove leek data?
                       return x.data.usedSubsystems & subsystemsToBreak;
                     }));
}

void runMacro(macroData *macroToRun) {
  // End all macros that conflict in subsystem usage
  breakMacros(macroToRun->usedSubsystems);
  // TODO: Add macro mutexs
  // Run the macro
  runningMacro newMacro;
  newMacro.data = *macroToRun;
  newMacro.macroTask =
      new pros::Task(newMacro.data.macroFunction, NULL, TASK_PRIORITY_DEFAULT,
                     TASK_STACK_DEPTH_DEFAULT, "");
  runningMacros.emplace_back(newMacro);
}

std::vector<macroData> getRunningMacroData() {
  std::vector<macroData> dataReturn;
  for (std::vector<runningMacro>::iterator vectorItter = runningMacros.begin();
       vectorItter != runningMacros.end(); vectorItter++)
    dataReturn.emplace_back(*vectorItter);
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
