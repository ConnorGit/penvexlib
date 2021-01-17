/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// TODO: Understand how function ordering affects a builder or at least why the
// position of withOdomitry affects its bool
#include "penvexlib/mods/asyncMeshMpPpControllerBuilder.hpp"

namespace okapi {
AsyncMotionProfileControllerBuilderMod::AsyncMotionProfileControllerBuilderMod(
    const std::shared_ptr<Logger> &ilogger)
    : logger(ilogger) {}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    const Motor &ioutput, const QLength &idiameter,
    const AbstractMotor::GearsetRatioPair &ipair) {
  return withOutput(std::make_shared<Motor>(ioutput), idiameter, ipair);
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    const MotorGroup &ioutput, const QLength &idiameter,
    const AbstractMotor::GearsetRatioPair &ipair) {
  return withOutput(std::make_shared<MotorGroup>(ioutput), idiameter, ipair);
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    const std::shared_ptr<ControllerOutput<double>> &ioutput,
    const QLength &idiameter, const AbstractMotor::GearsetRatioPair &ipair) {
  hasOutput = true;
  hasModel = false;
  output = ioutput;
  diameter = idiameter;
  pair = ipair;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    ChassisController &icontroller) {
  return withOutput(icontroller.getModel(), icontroller.getChassisScales(),
                    icontroller.getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    const std::shared_ptr<ChassisController> &icontroller) {
  return withOutput(icontroller->getModel(), icontroller->getChassisScales(),
                    icontroller->getGearsetRatioPair());
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOutput(
    const std::shared_ptr<ChassisModel> &imodel, const ChassisScales &iscales,
    const AbstractMotor::GearsetRatioPair &ipair) {
  hasOutput = false;
  hasModel = true;
  model = imodel;
  scales = iscales;
  pair = ipair;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withOdometry(
    const std::shared_ptr<Odometry> &iodometry,
    PurePursuitConstants *iPpConstants) {
  hasOdometry = true;
  odometry = iodometry;
  pursuitConstants = iPpConstants;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withLimits(
    const PathfinderLimits &ilimits) {
  hasLimits = true;
  limits = ilimits;
  return *this;
}

// AsyncMotionProfileControllerBuilderMod &
// AsyncMotionProfileControllerBuilderMod::withPpConstants() {
//   // hasPpConstants = true;
//   // PpConstants = iPpConstants;
//   return *this;
// }

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withTimeUtilFactory(
    const TimeUtilFactory &itimeUtilFactory) {
  timeUtilFactory = itimeUtilFactory;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::withLogger(
    const std::shared_ptr<Logger> &ilogger) {
  controllerLogger = ilogger;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::parentedToCurrentTask() {
  isParentedToCurrentTask = true;
  return *this;
}

AsyncMotionProfileControllerBuilderMod &
AsyncMotionProfileControllerBuilderMod::notParentedToCurrentTask() {
  isParentedToCurrentTask = false;
  return *this;
}

std::shared_ptr<AsyncLinearMotionProfileController>
AsyncMotionProfileControllerBuilderMod::buildLinearMotionProfileController() {
  if (!hasOutput) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No output given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  auto out = std::make_shared<AsyncLinearMotionProfileController>(
      timeUtilFactory.create(), limits, output, diameter, pair,
      controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK &&
      NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<AsyncLinearMotionProfileControllerMod>
AsyncMotionProfileControllerBuilderMod::
    buildLinearMotionProfileControllerMod() {
  if (!hasOutput) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No output given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  auto out = std::make_shared<AsyncLinearMotionProfileControllerMod>(
      timeUtilFactory.create(), limits, output, diameter, pair,
      controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK &&
      NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<AsyncMotionProfileController>
AsyncMotionProfileControllerBuilderMod::buildMotionProfileController() {
  if (!hasModel) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No model given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  auto out = std::make_shared<AsyncMotionProfileController>(
      timeUtilFactory.create(), limits, model, scales, pair, controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK &&
      NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<AsyncMeshMpPpController>
AsyncMotionProfileControllerBuilderMod::buildMeshMpPpController() {
  if (!hasModel) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No model given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasOdometry) {
    std::string msg(
        "AsyncMotionProfileControllerBuilderMod: No odometry given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!hasLimits) {
    std::string msg("AsyncMotionProfileControllerBuilderMod: No limits given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  // if (!hasPpConstants) {
  //   std::string msg("AsyncMotionProfileControllerBuilderMod: No Pure Pursuit
  //   "
  //                   "constants given.");
  //   LOG_ERROR(msg);
  //   throw std::runtime_error(msg);
  // }

  auto out = std::make_shared<AsyncMeshMpPpController>(
      timeUtilFactory.create(), limits, pursuitConstants, model, scales, pair,
      odometry, controllerLogger);
  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK &&
      NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}
} // namespace okapi
