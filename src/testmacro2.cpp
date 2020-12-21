/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex::macro;

namespace scripts {

/**
 * The macro function to run.
 */
void macroTest2(void *) {
  printf("\nstarting macroTest2");
  while (true) {
    printf("\nrunning macroTest2");
    pros::Task::delay(300);
  }
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned by annother task.
 */

macroData macroTest2_data{0b10, macroTest2, true};

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
