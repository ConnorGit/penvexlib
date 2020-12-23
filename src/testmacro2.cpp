/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex::macro;
using namespace okapi::literals;

namespace scripts {

/**
 * The identifacation of the subsystems running in this macro - must not be 0.
 */
const unsigned int used_subsystems = 0b10;

/**
 * The macro function to run.
 */
void macroTest2(void *) {

  profileBaseController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
  profileBaseController->setTarget("A");
  profileBaseController->waitUntilSettled();

  // Must have this line at the end of the macro
  endMacro(used_subsystems);
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned by annother task.
 */

macroData macroTest2_data{used_subsystems, macroTest2, true};

/**
 * THis function will run the macro.
 */
void runMacroTest2() { runMacro(&macroTest2_data); }

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initMacroTest2() { initMacro(&macroTest2_data); }

} // namespace scripts
