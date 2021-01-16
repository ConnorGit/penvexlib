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
const int RECORD_TIME = 5000;  // Period of time of one recording
const int MSEC_PER_FRAME = 10; // the time between frimes in msec
const int NUMBER_OF_FRAMES = (int)(RECORD_TIME / MSEC_PER_FRAME);
const int BYTES_RECORDED_PER_FRAME = 32; // 6 * 8;
const int RECORDED_DATA_SIZE = NUMBER_OF_FRAMES * BYTES_RECORDED_PER_FRAME / 4;
const std::string pathId = "testRec"; // TODO: add system for rec names

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
    double baseX[NUMBER_OF_FRAMES];
    double baseY[NUMBER_OF_FRAMES];
    double baseLV[NUMBER_OF_FRAMES];
    double baseRV[NUMBER_OF_FRAMES];
    // double intakeV[NUMBER_OF_FRAMES];
    // double conveyorV[NUMBER_OF_FRAMES];

    okapi::Rate timer;

    for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
      okapi::OdomState currentPos = base->getState();
      baseX[i] = currentPos.x.getValue();
      baseY[i] = currentPos.y.getValue();
      baseLV[i] = baseFL->getActualVelocity();
      baseRV[i] = baseFR->getActualVelocity();
      // intakeV[i] = ;
      // conveyorV[i] = ;
      timer.delayUntil(MSEC_PER_FRAME);
    }
    printf("Finished recording.\n");

    // POST PROCESSING:

    // Allocate memory
    int bufferSize = sizeof(Segment) * NUMBER_OF_FRAMES;
    std::unique_ptr<Segment, void (*)(void *)> leftTrajectory(
        (Segment *)malloc(bufferSize), free);
    std::unique_ptr<Segment, void (*)(void *)> rightTrajectory(
        (Segment *)malloc(bufferSize), free);
    std::unique_ptr<Segment, void (*)(void *)> baseTrajectory(
        (Segment *)malloc(bufferSize), free);

    for (int i = 0; i < NUMBER_OF_FRAMES; i++) {
      baseLV[i] /= 91.1409838751; // (PI*wheelDiam(m)*gearRatio/60sec))^-1
      baseRV[i] /= 91.1409838751;
      Segment baseLSeg = {((double)MSEC_PER_FRAME / 1000.0),
                          0.0,
                          0.0,
                          0.0,
                          baseLV[i],
                          0.0,
                          0.0,
                          0.0};
      (leftTrajectory.get())[i] = baseLSeg;
      Segment baseRSeg = {((double)MSEC_PER_FRAME / 1000.0),
                          0.0,
                          0.0,
                          0.0,
                          baseRV[i],
                          0.0,
                          0.0,
                          0.0};
      (rightTrajectory.get())[i] = baseRSeg;
      Segment baseSeg = {
          ((double)MSEC_PER_FRAME / 1000.0), baseX[i], baseY[i], 0.0,
          (baseRV[i] + baseLV[i]) / 2.0,     0.0,      0.0,      0.0};
      (baseTrajectory.get())[i] = baseSeg;
    }

    profileBaseController->takePath(leftTrajectory, rightTrajectory,
                                    baseTrajectory, NUMBER_OF_FRAMES, pathId);
    profileBaseController->storePath("/data/recordings/", pathId);
    printf("Saved recording.\n");

    breakMacros(used_subsystems);
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
