/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Test2
 */

#include "main.h"

using namespace penvex;
using namespace okapi::literals;

namespace scripts::test2Macro {

/**
 * The identifacation of the subsystems running in this macro.
 */
const unsigned int used_subsystems = BASE;

/**
 * The macro function to run.
 */
void macro_func(void *) {

  printf("Started test2 macro\n");

  // Must have this line at the end of restart macro
  macro->remove();
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned by annother task.
 */

Macro *macro;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void init() {
  macro = new Macro(used_subsystems, macro_func, true);
  macro->init();
}

/**
 * Begin the macro's execution
 */
void run() { macro->run(); }

/**
 * Remove the marco from existing macros and free its data
 */
void remove() { macro->remove(); }

} // namespace scripts::test2Macro
