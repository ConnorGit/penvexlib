/*
 * This contains the code and identifacation for a single macro segment
 * described below:
 *
 * Attempt to symplisticaly balance the robot on a sea saw
 */

#include "main.h"

using namespace penvex;
using namespace okapi::literals;

namespace scripts::balanceMacro {

/**
 * The identifacation of the subsystems running in this macro.
 */
const unsigned int used_subsystems = BASE;

/**
 * Macro Data that needs to be acessed by other tasks
 * Needs to be freed by the remove() function
 */
okapi::IterativePosPIDController *turnController;
okapi::IterativePosPIDController *feedForwardAngVController;
okapi::IterativePosPIDController *inclanationController;

// Call this when dealing with variables internal to the controllers so large
// jumps and unexpected behavior is avoided
pros::Mutex controllerMutex;

/**
 * The macro function to run.
 */
void macro_func(void *) {

  printf("Started balance macro\n");

  okapi::Rate rate;
  okapi::QTime sampleTime{10_ms};

  // NOTE: Correct this
  double reversed = 1.0;

  // Configure the controllers
  controllerMutex.take(TIMEOUT_MAX);
  turnController->setSampleTime(sampleTime);
  feedForwardAngVController->setSampleTime(sampleTime);
  inclanationController->setSampleTime(sampleTime);

  // Configure this
  feedForwardAngVController->setControllerSetTargetLimits(1.0, -1.0);
  controllerMutex.give();

  double currentAngle;
  double currentAngularVleocity;

  while (true) {
    // NOTE: Could eat up cpu time - untested
    controllerMutex.take(TIMEOUT_MAX);

    double currentAngle = imuY->controllerGet();
    double currentAngularVleocity =
        (currentAngle - turnController->getProcessValue()) /
        sampleTime.convert(okapi::second);

    // All the controllers return a range of -1.0 to 1.0
    // Controllerset reinterprets that range

    feedForwardAngVController->controllerSet(
        inclanationController->step(currentAngle));

    double linearSpeed =
        feedForwardAngVController->step(currentAngularVleocity);
    double turnSpeed = turnController->step(imuY->controllerGet());

    controllerMutex.give();

    base->getModel()->driveVector(linearSpeed * reversed, turnSpeed);

    rate.delayUntil(sampleTime);
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

Macro *macro;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 */
void init() {

  controllerMutex.take(TIMEOUT_MAX);

  turnController = new okapi::IterativePosPIDController(
      0.0013, 0.0003, 0.00002, 0.0, okapi::TimeUtilFactory::createDefault());
  feedForwardAngVController = new okapi::IterativePosPIDController(
      0.1, 0.0, 0.1, 0.0, okapi::TimeUtilFactory::createDefault());
  inclanationController = new okapi::IterativePosPIDController(
      0.1, 0.0, 0.1, 0.0, okapi::TimeUtilFactory::createDefault());

  inclanationController->setTarget(0.0);
  turnController->setTarget(0.0);

  controllerMutex.give();

  macro = new Macro(used_subsystems, macro_func, true); // Is a restart macro
  macro->init();
}

/**
 * Begin the macro's execution
 */
void run() {
  controllerMutex.take(TIMEOUT_MAX);
  turnController->reset();
  feedForwardAngVController->reset();
  inclanationController->reset();
  controllerMutex.give();

  macro->run();
}

/**
 * Remove the marco from existing macros and free its data
 */
void remove() {
  controllerMutex.take(TIMEOUT_MAX);
  delete turnController;
  delete feedForwardAngVController;
  delete inclanationController;
  controllerMutex.give();

  macro->remove();
}

} // namespace scripts::balanceMacro
