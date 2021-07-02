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
const unsigned int used_subsystems = BASE | INTAKE | CONVEYOR;

std::string dirName = "/data/recordings/";

const int autonPathCount = 4;
std::tuple<std::string, penvex::record::subsystemPathFileData>
    autonPaths[autonPathCount];

void initAutonPathArray() {
  autonPaths[0] = {"Rec1.base", basePDat};
#define A1_B_AB autonPaths[0]
  autonPaths[1] = {"Rec1.intake", intakePDat};
#define A1_I_UP_1 autonPaths[1]
  autonPaths[2] = {"Rec2.base", basePDat};
#define A1_B_BC autonPaths[2]
  autonPaths[3] = {"Rec2.intake", intakePDat};
#define A1_I_DWN autonPaths[3]
}

void stopAuto() {
  while (true)
    pros::Task::delay(10000);
}

/**
 * The macro function to run.
 */
void macroTest2_func(void *) {

  printf("started macro\n");

  // resetData();
  /*
  ///////////////////BALL 1 2

  ld(B, "Rec2.base");
  ld(I, "Rec2.intake");
  ld(C, "Rec2.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec2.base", okapi::AsyncMeshMpPpController::Pp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec2.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec2.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec2.base");

  profileIntakeController->removePath("Rec2.intake");

  profileConveyorController->removePath("Rec2.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();

  // stopAuto();

  ////////// GET TO TOWER 1

  ld(B, "Rec11.base");
  ld(I, "Rec11.intake");
  ld(C, "Rec11.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec11.base", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec11.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec11.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec11.base");

  profileIntakeController->removePath("Rec11.intake");

  profileConveyorController->removePath("Rec11.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();

  ////////// TOWER 1 descore and score

  ld(B, "Rec33.base");
  ld(I, "Rec33.intake");
  ld(C, "Rec33.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec33.base", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec33.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec33.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec33.base");

  profileIntakeController->removePath("Rec33.intake");

  profileConveyorController->removePath("Rec33.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();

  base->getModel()->forward(100);
  pros::Task::delay(200);
  base->stop();

  // // // ////////// TOWER 1 back up and reset with turn

  towerResetOdom('I');

  ld(B, "Rec32.base");
  ld(I, "Rec32.intake");
  ld(C, "Rec32.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec32.base", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec32.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec32.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec32.base");

  profileIntakeController->removePath("Rec32.intake");

  profileConveyorController->removePath("Rec32.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();

  // ////////// PID to ball 3

  intake->moveVelocity(200);

  base->driveToPoint({1.857282_m, 1.76878_m}); //{1.867282_m, 1.66878_m}
  base->stop();

  intake->moveVelocity(0);

  // ////////// turn and ball 4 to tower 2

  base->turnToPoint({1.8028_m, 0.39605_m});

  ld(B, "Rec21.base");
  ld(I, "Rec21.intake");
  // ld(C, "Rec21.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec21.base", okapi::AsyncMeshMpPpController::Pp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec21.intake");
  profileIntakeController->flipDisable(false);

  // profileConveyorController->setTarget("Rec21.conveyor");
  // profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  // profileConveyorController->waitUntilSettled();

  profileBaseController->flipDisable(true);
  base->stop();

  profileBaseController->removePath("Rec21.base");

  profileIntakeController->removePath("Rec21.intake");

  // profileConveyorController->removePath("Rec21.conveyor");

  printf("done\n");

  profileBaseController->flipDisable(true);
  base->stop();

  // ////////// score tower 2

  ld(B, "Rec22.base");
  ld(I, "Rec22.intake");
  ld(C, "Rec22.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec22.base", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec22.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec22.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec22.base");

  profileIntakeController->removePath("Rec22.intake");

  profileConveyorController->removePath("Rec22.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();

  // ////////// ALL TOWER THREE !!!!!!!!!!!!!!

  towerResetOdom('H');

  ld(B, "Rec23.base");
  ld(I, "Rec23.intake");
  ld(C, "Rec23.conveyor");

  // ldTmp();

  profileBaseController->setTarget(
      "Rec23.base", okapi::AsyncMeshMpPpController::Mp, false, false);
  profileBaseController->flipDisable(false);

  profileIntakeController->setTarget("Rec23.intake");
  profileIntakeController->flipDisable(false);

  profileConveyorController->setTarget("Rec23.conveyor");
  profileConveyorController->flipDisable(false);

  profileBaseController->waitUntilSettled();

  profileIntakeController->waitUntilSettled();

  profileConveyorController->waitUntilSettled();

  profileBaseController->removePath("Rec23.base");

  profileIntakeController->removePath("Rec23.intake");

  profileConveyorController->removePath("Rec23.conveyor");

  profileBaseController->flipDisable(true);
  base->stop();
  */

  ////////////////////////////////////////////////////////////////////////////////////

  // penvex::master::runMasterFile("/data/recordings/temp/", "temp");

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
