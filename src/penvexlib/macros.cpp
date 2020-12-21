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
  const macroData
      *data; // This stores address of the struct with the subsystem ids and
             // function pointers for the currently stored macro in the task
  pros::Task
      *macroTask; // This is one of the four tasks initalized to run macros

  // NOTE: I was dumb when I was first writing all this code because I didn't
  // understand the benifits of emplace back,
  // essentailly use this constructor whaen initalizing vector elements
  // with that function to avoid copying temporary data.
  runningMacro(const macroData *data, pros::Task *macroTask)
      : data(data), macroTask(macroTask) {
    printf("Making\n");
  }

  ~runningMacro() {
    printf("Destructing\n");
    // macroTask->remove(); // TODO: Does remove leek data, I would think
    //                      // that it wouldnt but should prob test?
  }
};

/**
 * A list containing every currently running macro and its data.
 */
std::vector<runningMacro> runningMacros;

void init() {
  // NOTE: Prob never more than three macros will run at once so there souldn't
  // be any performance problems.
  runningMacros.reserve(3);
}

void breakMacros(unsigned int subsystemsToBreak) {
  printf("REmoving\n");
  printf("%d\n", runningMacros.size());
  runningMacroMutex.take(TIMEOUT_MAX);

  // Bitwise & used as a mask for usedSubsustems
  runningMacros.erase(std::remove_if(runningMacros.begin(), runningMacros.end(),
                                     [subsystemsToBreak](runningMacro x) {
                                       return x.data->usedSubsystems &
                                              subsystemsToBreak;
                                     }));

  runningMacroMutex.give();
  printf("%d\n", runningMacros.size());
}

void runMacro(const macroData *macroToRun, void *macroFuncParams) {
  // You cant run a macro with no subsystems
  if (macroToRun->usedSubsystems == 0b0) {
    printf("Failed to run macro with no subsystems.");
    return;
  }

  // End all macros that conflict in subsystem usage
  // breakMacros(macroToRun->usedSubsystems);
  // Run the macro
  printf("%d\n", runningMacros.size());
  pros::Task *newTask = new pros::Task(
      macroToRun->macroFunction, macroFuncParams, macroToRun->prio,
      macroToRun->stack_depth, macroToRun->name);
  printf("Creating\n");
  runningMacroMutex.take(TIMEOUT_MAX);
  runningMacros.emplace_back(macroToRun, newTask);
  runningMacroMutex.give();
  printf("%d\n", runningMacros.size());
}

std::vector<macroData> getRunningMacroData() {
  std::vector<macroData> dataReturn;
  for (std::vector<runningMacro>::iterator vectorItter = runningMacros.begin();
       vectorItter != runningMacros.end(); ++vectorItter)
    dataReturn.emplace_back(*(vectorItter->data));
  return dataReturn;
}

unsigned int getAllUsedSubsystems() {
  unsigned int allUsedSubsystem = 0b0;
  for (std::vector<runningMacro>::iterator vectorItter = runningMacros.begin();
       vectorItter != runningMacros.end(); ++vectorItter) {
    printf("%d\n", (*vectorItter).data->usedSubsystems);
    allUsedSubsystem |= (*vectorItter).data->usedSubsystems;
  }
  return allUsedSubsystem;
}

} // namespace penvex::macro
