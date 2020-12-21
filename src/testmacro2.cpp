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
void macroTest2(void *) {
  while (true) {
    printf("\nrunning macroTest2");
    pros::Task::delay(100);
  }
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 */
const macroData macroTest2_data{0b100, macroTest2};

/**
 * THis function will run the macro.
 */
void runMacroTest2() { runMacro(&macroTest2_data); }
