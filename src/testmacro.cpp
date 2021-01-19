/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex::macro;
using namespace okapi;

namespace scripts {

/**
 * The macro function to run.
 */
void macroTest(void *) {
  printf("\nstarting macroTest");
  while (true) {
    base->turnToAngle(90_deg);
    pros::Task::delay(5000);
    // printf("%f\n", baseFL->getPosition());
    // OdomState state = base->getOdometry()->getState();
    // printf("%f, %f %f\n", state.x.convert(inch), state.y.convert(inch),
    //        state.theta.convert(degree));
    // pros::Task::delay(20);
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
macroData macroTest_data{0b01, macroTest, false};

/**
 * This function will run the macro.
 */
void runMacroTest() { runMacro(&macroTest_data); }

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initMacroTest() { initMacro(&macroTest_data); }

} // namespace scripts
