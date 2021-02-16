/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Macro Test
 */

#include "main.h"

using namespace penvex;
using namespace okapi::literals;

namespace scripts {

/**
 * The identifacation of the subsystems running in this macro - must not be 0.
 */
const unsigned int used_subsystems = 0b10;

/**
 * The macro function to run.
 */
void macroTest2_func(void *) {

  printf("started macro\n");

  penvex::master::runMasterFile("/data/recordings/temp/", "temp");

  // // penvex::record::loadPath(profileBaseController, "/data/recordings/",
  // //                          "testRec.base", "testRec");
  // // penvex::record::loadPath(profileIntakeController, "/data/recordings/",
  // //                          "testRec.intake", "testRec");
  // // penvex::record::loadPath(profileConveyorController, "/data/recordings/",
  // //                          "testRec.conveyor", "testRec");
  //
  // profileBaseController->generatePath(
  //     {{0.0_in, 0.0_in, 0.0_deg}, {20.0_in, 10.0_in, 0.0_deg}}, "testRec");
  //
  // printf("started driving\n");
  // profileBaseController->setTarget(
  //     "testRec", okapi::AsyncMeshMpPpController::Pp, false, false);
  //
  // // profileIntakeController->setTarget("testRec");
  // //
  // // profileConveyorController->setTarget("testRec");
  //
  // profileBaseController->waitUntilSettled();
  // // profileIntakeController->waitUntilSettled();
  // // profileConveyorController->waitUntilSettled();
  // base->stop();
  // // intake->moveVoltage(0);
  // // conveyor->moveVoltage(0);
  //
  // printf("done with profile\n");

  // Must have this line at the end of the macro
  macroTest2->remove();
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned by annother task.
 */

Macro *macroTest2;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initMacroTest2() {
  macroTest2 = new Macro(used_subsystems, macroTest2_func, true);
  macroTest2->init();
}

} // namespace scripts
