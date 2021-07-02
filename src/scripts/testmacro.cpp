/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex;
using namespace okapi;

namespace scripts {

/**
 * The macro function to run.
 */
void macroTest_func(void *) {
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
 * The macro object which contains both the macro function to
 * run in its own task and an identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned.
 */
Macro *macroTest;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initMacroTest() {
  macroTest = new Macro(0b10000, macroTest_func, false);
  macroTest->init();
}

} // namespace scripts
