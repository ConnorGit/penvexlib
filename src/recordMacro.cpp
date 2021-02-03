/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Records the movemnt of the robot and passes it to path controllers.
 */

#include "main.h"

using namespace penvex::macro;
using namespace okapi::literals;

namespace penvex::record {

// STICK THIS IN A DIFF FILE
const int RECORD_TIME = 20000; // Period of time of one recording
const int MSEC_PER_FRAME = 10; // the time between frimes in msec
const int NUMBER_OF_FRAMES = (int)(RECORD_TIME / MSEC_PER_FRAME);
const int BYTES_RECORDED_PER_FRAME = 48; // 6 * 8;
const int RECORDED_DATA_SIZE = NUMBER_OF_FRAMES * BYTES_RECORDED_PER_FRAME / 4;
const std::string recDir = "/data/recordings/";
const std::string recTempDir = recDir + "temp/";

/**
 * The identifacation of the subsystems running in this macro - must not be 0.
 */
const unsigned int used_subsystems = 0b1000;

/**
 * The macro function to run.
 */
void recordLoop(void *) {
  while (true) {
    printf("Stared recording.\n");
    pros::lcd::print(0, "Recording %fs remaining.",
                     ((double)RECORD_TIME) / 1000.0);

    // resetData();

    okapi::ControllerButton buttonX(okapi::ControllerDigital::X);
    okapi::ControllerButton buttonY(okapi::ControllerDigital::Y);

    double baseX[NUMBER_OF_FRAMES];
    double baseY[NUMBER_OF_FRAMES];
    double baseLV[NUMBER_OF_FRAMES];
    double baseRV[NUMBER_OF_FRAMES];
    double intakeV[NUMBER_OF_FRAMES];
    double conveyorV[NUMBER_OF_FRAMES];

    okapi::Rate timer;

    int lengthOfRec = NUMBER_OF_FRAMES;

    for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
      okapi::OdomState currentPos = base->getState();
      baseX[i] = currentPos.x.getValue();
      baseY[i] = currentPos.y.getValue();
      baseLV[i] = baseFL->getActualVelocity();
      baseRV[i] = baseFR->getActualVelocity();
      intakeV[i] = intake->getActualVelocity();
      conveyorV[i] = conveyor->getActualVelocity();
      if (buttonX.changedToPressed()) {
        lengthOfRec = (i + 1);
        break;
      }
      // printf("%f\n", conveyorV[i]);
      if (i % 10 == 0)
        pros::lcd::print(0, "Recording %fs remaining.",
                         ((double)(RECORD_TIME - (i * MSEC_PER_FRAME))) /
                             1000.0);
      timer.delayUntil(MSEC_PER_FRAME);
    }
    printf("Finished recording.\n");

    // POST PROCESSING:

    double dt = ((double)MSEC_PER_FRAME / 1000.0);

    for (int i = 0; i < lengthOfRec; i++) {
      baseLV[i] *= 0.00365733744; // (PI*wheelDiam(m)*gearRatio(1)/60sec))
      baseRV[i] *= 0.00365733744;
      intakeV[i] *= 0.00465479311 * 1.0481;
      conveyorV[i] *= 0.00398982267 * 5.9568944116; // hack
    }

    // //////SAVE_RAW_DATA//////
    // // Allocate memory
    // int bufferSize = sizeof(Segment) * lengthOfRec;
    // std::unique_ptr<Segment, void (*)(void *)> leftTrajectory(
    //     (Segment *)malloc(bufferSize), free);
    // std::unique_ptr<Segment, void (*)(void *)> rightTrajectory(
    //     (Segment *)malloc(bufferSize), free);
    // std::unique_ptr<Segment, void (*)(void *)> baseTrajectory(
    //     (Segment *)malloc(bufferSize), free);
    // std::unique_ptr<Segment, void (*)(void *)> intakeTrajectory(
    //     (Segment *)malloc(bufferSize), free);
    // std::unique_ptr<Segment, void (*)(void *)> conveyorTrajectory(
    //     (Segment *)malloc(bufferSize), free);
    //
    // for (int i = 0; i < lengthOfRec; i++) {
    //   Segment baseLSeg = {dt, 0.0, 0.0, 0.0, baseLV[i], 0.0, 0.0, 0.0};
    //   (leftTrajectory.get())[i] = baseLSeg;
    //   Segment baseRSeg = {dt, 0.0, 0.0, 0.0, baseRV[i], 0.0, 0.0, 0.0};
    //   (rightTrajectory.get())[i] = baseRSeg;
    //   Segment baseSeg = {
    //       dt,  baseX[i], baseY[i], 0.0, (baseRV[i] + baseLV[i]) / 2.0,
    //       0.0, 0.0,      0.0};
    //   (baseTrajectory.get())[i] = baseSeg;
    //   Segment intakeSeg = {dt, 0.0, 0.0, 0.0, intakeV[i], 0.0, 0.0, 0.0};
    //   (intakeTrajectory.get())[i] = intakeSeg;
    //   Segment conveyorSeg = {dt, 0.0, 0.0, 0.0, conveyorV[i], 0.0, 0.0, 0.0};
    //   (conveyorTrajectory.get())[i] = conveyorSeg;
    // }
    //
    // profileBaseController->takePath(leftTrajectory, rightTrajectory,
    //                                 baseTrajectory, lengthOfRec,
    //                                 "temp.raw");
    // profileIntakeController->takePath(intakeTrajectory, lengthOfRec,
    //                                   "temp.raw");
    // profileConveyorController->takePath(conveyorTrajectory, lengthOfRec,
    //                                     "temp.raw");

    /////////

    double dtArr[lengthOfRec];
    std::fill_n(dtArr, lengthOfRec, dt);

    const double *baseData[] = {dtArr, baseX, baseY, baseLV, baseRV};

    const double *intakeData[] = {dtArr, intakeV};

    const double *conveyerData[] = {dtArr, conveyorV};

    // Save all the data temporarily to avoid mistake
    createBasicMasterFile(recTempDir, "temp");
    storeDoubles(lengthOfRec, 5, baseData, recTempDir, "temp.base");
    storeDoubles(lengthOfRec, 2, intakeData, recTempDir, "temp.intake");
    storeDoubles(lengthOfRec, 2, conveyerData, recTempDir, "temp.conveyor");

    printf("Save recording? (Y/X)\n");
    pros::lcd::print(0, "Save recording? (Y/X)");

    while (true) {
      if (buttonY.changedToPressed()) {

        const std::string name = getNewRecID(recDir);

        printf("Saving...\n");
        pros::lcd::print(0, "Saving as %s", name);
        pros::Task::delay(800);

        createBasicMasterFile(recDir, name);

        storeDoubles(lengthOfRec, 5, baseData, recDir, name + ".base");
        storeDoubles(lengthOfRec, 2, intakeData, recDir, name + ".intake");
        storeDoubles(lengthOfRec, 2, conveyerData, recDir, name + ".conveyor");
        printf("Finished saving.\n");
        break;

      } else if (buttonX.changedToPressed()) {
        printf("NOT Saving Recording.\n");
        pros::lcd::print(0, "DELETING");
        pros::Task::delay(800);
        break;
      }
      pros::Task::delay(50);
    }

    pros::lcd::print(0, "");

    breakMacros(used_subsystems);
  }
}

/**
 * The identifacation for the macro which contains both the macro function to
 * run in its own task and a identifactaion for the subsystems running in it
 * to allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned by annother task.
 */

macroData recordMacro_data{used_subsystems,
                           recordLoop,
                           false,
                           TASK_PRIORITY_DEFAULT,
                           RECORDED_DATA_SIZE + TASK_STACK_DEPTH_DEFAULT,
                           "Record"};

/**
 * THis function will run the macro.
 */
void runRecordMacro() { runMacro(&recordMacro_data); }

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initRecordMacro() { initMacro(&recordMacro_data); }

} // namespace penvex::record
