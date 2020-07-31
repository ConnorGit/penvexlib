/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex::macro;

/**
 * The macro function to run.
 */
void macroTest(void *) {
  while (true) {
    printf("\nrunning macroTest");
    pros::Task::delay(100);
  }
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 */
macroData *macroTest_data = new macroData{macroTest, 0b11};

/**
 * THis function will run the macro.
 */
void runMacroTest() { runMacro(macroTest_data); }
