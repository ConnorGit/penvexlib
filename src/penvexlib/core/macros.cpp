/*
 * \file penvexlib/macros.cpp
 *
 * Provides macro functionality.
 */

#include "main.h"
#include <algorithm>
#include <cstring>

namespace penvex {

unsigned int Macro::numberOfSubsystems = 0;
Macro **Macro::runningMacros;
pros::Mutex Macro::runningMacroMutex;

Macro::Macro(unsigned int iusedSubsystems, void (*imacroFunction)(void *),
             bool irestartOnBreak, void *ifuncParams, std::uint32_t iprio,
             std::uint16_t istack_depth, const char *iname)
    : usedSubsystems(iusedSubsystems), macroFunction(imacroFunction),
      restartOnBreak(irestartOnBreak), funcParams(ifuncParams), prio(iprio),
      stack_depth(istack_depth), name(iname), macroTask(nullptr) {
  if (this->usedSubsystems == 0b0) {
    printf("Failed to make macro with no subsystems.");
    throw std::invalid_argument("Failed to make macro with no subsystems.");
  }
}

void Macro::initRunner(unsigned int inumberOfSubsystems) {
  runningMacroMutex.take(TIMEOUT_MAX);
  numberOfSubsystems = inumberOfSubsystems;
  runningMacros = new Macro *[numberOfSubsystems];
  for (int i = 0; i < numberOfSubsystems; i++)
    runningMacros[i] = nullptr;
  runningMacroMutex.give();
}

void Macro::breakMacros(unsigned int subsystemsToBreak) {
  runningMacroMutex.take(TIMEOUT_MAX);

  // Bitwise & used as a mask for usedSubsustems
  for (int i = 0; i < numberOfSubsystems; i++)
    if ((runningMacros[i] != nullptr) &&
        (runningMacros[i]->usedSubsystems & subsystemsToBreak)) {

      Macro *macroToBreak = runningMacros[i];

      runningMacros[i] = nullptr;

      macroToBreak->macroTask->suspend();

      // NOTE: WARNING MAKE SURE NOT TO LEAK MEMORY ON REMOVED TASKS
      if (macroToBreak->restartOnBreak)
        macroToBreak->macroTask->remove();
    }

  runningMacroMutex.give();
}

void Macro::run(void *macroFuncParams) {
  this->funcParams = macroFuncParams;

  // End all macros that conflict in subsystem usage
  breakMacros(this->usedSubsystems);
  // Run the macro
  if ((this->macroTask != nullptr) &&
      this->macroTask->get_state() == pros::E_TASK_STATE_SUSPENDED)
    this->macroTask->resume();
  else
    this->macroTask = new pros::Task(this->macroFunction, NULL, this->prio,
                                     this->stack_depth, this->name);

  // Add pointer to the the started macro tho the list of running macros
  runningMacroMutex.take(TIMEOUT_MAX);
  for (int i = 0; i < numberOfSubsystems; i++)
    if (runningMacros[i] == nullptr) {
      runningMacros[i] = this;
      break;
    }
  runningMacroMutex.give();
}

void Macro::init() {
  this->macroTask->set_priority(TASK_PRIORITY_MIN);

  this->run();
  this->macroTask->suspend();

  this->macroTask->set_priority(this->prio);

  runningMacroMutex.take(TIMEOUT_MAX);
  for (int i = 0; i < numberOfSubsystems; i++)
    if (runningMacros[i] == this)
      runningMacros[i] = nullptr;
  runningMacroMutex.give();
}

void Macro::remove() {
  if (!this->restartOnBreak)
    printf("WARNING: Non-restart macros should never be removed from the "
           "running macros this way.");

  runningMacroMutex.take(TIMEOUT_MAX);
  // Bitwise & used as a mask for usedSubsustems
  for (int i = 0; i < numberOfSubsystems; i++)
    if ((runningMacros[i] != nullptr) &&
        (runningMacros[i]->usedSubsystems & this->usedSubsystems)) {

      runningMacros[i] = nullptr;
    }

  runningMacroMutex.give();
}

Macro **Macro::getRunningMacroData(Macro *fillArr[]) {
  runningMacroMutex.take(TIMEOUT_MAX);
  for (int i = 0; i < numberOfSubsystems; i++)
    fillArr[i] = runningMacros[i];
  runningMacroMutex.give();
  return fillArr;
}

unsigned int Macro::getAllUsedSubsystems() {
  runningMacroMutex.take(TIMEOUT_MAX);
  unsigned int allUsedSubsystem = 0b0;
  for (int i = 0; i < numberOfSubsystems; i++)
    if (runningMacros[i] != nullptr)
      allUsedSubsystem |= runningMacros[i]->usedSubsystems;
  runningMacroMutex.give();
  return allUsedSubsystem;
}

unsigned int Macro::getUsedSubsystems() { return this->usedSubsystems; }

std::uint32_t Macro::getPriority() { return this->macroTask->get_priority(); }

void Macro::setPriority() { this->macroTask->set_priority(this->prio); }

void Macro::setPriority(std::uint32_t prio) {
  this->macroTask->set_priority(prio);
}

} // namespace penvex
