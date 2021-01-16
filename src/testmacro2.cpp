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
  // okapi::OdomState c = base->getOdometry()->getState();
  // profileBaseController->generatePath(
  // {{c.x, c.y, c.theta}, {c.x + 36_in, c.y + 36_in, c.theta}}, "A");
  profileBaseController->setTarget(
      "testRec", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->waitUntilSettled();
  base->stop();
  // profileBaseController->removePath("A");
  printf("done with profile\n");
  // profileBaseController->storePath("/data/default/", "A");
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
