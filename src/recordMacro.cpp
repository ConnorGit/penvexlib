/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Records the movemnt of the robot and passes it to path controllers.
 */

#include "main.h"

using namespace penvex;
using namespace okapi::literals;

namespace penvex::record {

// STICK THIS IN A DIFF FILE
const int RECORD_TIME = 60000; // Period of time of one recording
const int MSEC_PER_FRAME = 10; // the time between frimes in msec
const int NUMBER_OF_FRAMES = (int)(RECORD_TIME / MSEC_PER_FRAME);
const int BYTES_RECORDED_PER_FRAME = 72; // 9 * 8;
const int RECORDED_DATA_SIZE = NUMBER_OF_FRAMES * BYTES_RECORDED_PER_FRAME / 4;
const std::string recDir = "/data/recordings/";
const std::string recTempDir = recDir + "temp/";

/**
 * The identifacation of the subsystems running in this macro - must not be 0.
 */
const unsigned int used_subsystems = BIGDATA1;

/**
 * The macro function to run.
 */
void recordLoop(void *) {
  while (true) {
    printf("Stared record.\n");

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

    pros::lcd::print(0, "Recording for %.1fs", ((double)RECORD_TIME) / 1000.0);

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
      timer.delayUntil(MSEC_PER_FRAME);
    }

    printf("Finished recording.\n");

    // POST PROCESSING:

    double dt = ((double)MSEC_PER_FRAME / 1000.0);

    double dtBaseArr[lengthOfRec];
    std::fill_n(dtBaseArr, lengthOfRec, dt);

    double dtIntakeArr[lengthOfRec];
    std::fill_n(dtIntakeArr, lengthOfRec, dt);

    double dtConveyorArr[lengthOfRec];
    std::fill_n(dtConveyorArr, lengthOfRec, dt);

    // This seems slow but I dont think it matters because I have to do this
    // multiplacation anyway
    // for (int i = 0; i < lengthOfRec; i++) {
    //   baseLV[i] *= 0.00365733744; // (PI*wheelDiam(m)*gearRatio(1)/60sec))
    //   baseRV[i] *= 0.00365733744;
    //   intakeV[i] *= 0.00465479311 * 1.0481;
    //   conveyorV[i] *= 0.00398982267 * 5.9568944116; // hack
    // }

    auto processLambda = [lengthOfRec, dt](double *velArr, double *dtArr,
                                           long double scalar) {
      int finalArrPos = 0, i = 0;
      bool stopped = false;
      for (; i < lengthOfRec; i++) {
        if ((velArr[i]) == 0.0) {
          if (!stopped) {
            velArr[finalArrPos] = 0.0;
            finalArrPos++;
            stopped = true;
          }
        } else {
          if (stopped) {
            stopped = false;
            dtArr[finalArrPos - 1] = dt * ((double)(i - finalArrPos + 1));
          }

          velArr[finalArrPos] = velArr[i] * scalar;
          dtArr[finalArrPos] = dt;

          finalArrPos++;
        }
      }
      if (stopped)
        dtArr[finalArrPos - 1] = dt * ((double)(i - finalArrPos + 1));
      return finalArrPos;
    };

    auto processLambdaComplex =
        [lengthOfRec, dt](double *velArrs[], int numOfVels, double *extraArrs[],
                          int numOfExtras, double *dtArr, long double scalar) {
          int finalArrPos = 0, i = 0, j, k;
          bool stopped = false;
          for (; i < lengthOfRec; i++) {
            for (j = 0; j < numOfVels; j++) {
              if (velArrs[j][i] != 0.0)
                break;
            }

            if (j == numOfVels) {
              if (!stopped) {
                for (k = 0; k < numOfVels; k++) {
                  velArrs[k][finalArrPos] = 0.0;
                }
                for (k = 0; k < numOfExtras; k++) {
                  extraArrs[k][finalArrPos] = extraArrs[k][i];
                }
                finalArrPos++;
                stopped = true;
              }
            } else {
              if (stopped) {
                stopped = false;
                dtArr[finalArrPos - 1] = dt * ((double)(i - finalArrPos + 1));
              }

              for (k = 0; k < numOfVels; k++) {
                velArrs[k][finalArrPos] = velArrs[k][i] * scalar;
              }
              for (k = 0; k < numOfExtras; k++) {
                extraArrs[k][finalArrPos] = extraArrs[k][i];
              }
              dtArr[finalArrPos] = dt;

              finalArrPos++;
            }
          }
          if (stopped)
            dtArr[finalArrPos - 1] = dt * ((double)(i - finalArrPos + 1));
          return finalArrPos;
        };

    double *baseVelArrs[] = {baseLV, baseRV};
    double *baseXYArrs[] = {baseX, baseY};
    int baseLen = processLambdaComplex(
        baseVelArrs, 2, baseXYArrs, 2, dtBaseArr,
        0.00365733744); // (PI*wheelDiam(m)*gearRatio(1)/60sec))
    int intakeLen = processLambda(intakeV, dtIntakeArr, 0.00465479311 * 1.0481);
    int conveyorLen = processLambda(conveyorV, dtConveyorArr,
                                    0.00398982267 * 5.9568944116); // hack

    double startWait = dtBaseArr[0];
    if (dtIntakeArr[0] < startWait)
      startWait = dtIntakeArr[0];
    if (dtConveyorArr[0] < startWait)
      startWait = dtConveyorArr[0];
    startWait -= dt;

    dtBaseArr[0] -= startWait;
    dtIntakeArr[0] -= startWait;
    dtConveyorArr[0] -= startWait;

    dtBaseArr[baseLen - 1] = dt;
    dtIntakeArr[intakeLen - 1] = dt;
    dtConveyorArr[conveyorLen - 1] = dt;

    // printf("2 recording.\n");

    /////////

    const double *baseData[] = {dtBaseArr, baseX, baseY, baseLV, baseRV};

    const double *intakeData[] = {dtIntakeArr, intakeV};

    const double *conveyerData[] = {dtConveyorArr, conveyorV};

    // Save all the data temporarily to avoid mistake
    createBasicMasterFile(recTempDir, "temp");
    storeDoubles(baseLen, 5, baseData, recTempDir, "temp.base");
    storeDoubles(intakeLen, 2, intakeData, recTempDir, "temp.intake");
    storeDoubles(conveyorLen, 2, conveyerData, recTempDir, "temp.conveyor");

    printf("Save recording? (Y/X)\n");
    pros::lcd::print(0, "Save recording? (Y/X)");

    while (true) {
      if (buttonY.changedToPressed()) {

        const std::string name = getNewRecID(recDir);

        printf("Saving...\n");
        pros::lcd::print(0, "Saving as %s", name);
        pros::Task::delay(800);

        createBasicMasterFile(recDir, name);

        storeDoubles(baseLen, 5, baseData, recDir, name + ".base");
        storeDoubles(intakeLen, 2, intakeData, recDir, name + ".intake");
        storeDoubles(conveyorLen, 2, conveyerData, recDir, name + ".conveyor");
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

    Macro::breakMacros(used_subsystems);
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

Macro *recordMacro;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void initRecordMacro() {
  recordMacro =
      new Macro(used_subsystems, recordLoop, false, NULL, TASK_PRIORITY_DEFAULT,
                RECORDED_DATA_SIZE + TASK_STACK_DEPTH_DEFAULT, "Record");
  recordMacro->init();
}

} // namespace penvex::record
