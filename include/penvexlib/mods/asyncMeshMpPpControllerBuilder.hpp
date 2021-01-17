/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _MOTION_PROFILE_BUILDER_HPP_
#define _MOTION_PROFILE_BUILDER_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

#include "penvexlib/mods//asyncLinearMotionProfileControllerMod.hpp"
#include "penvexlib/mods/asyncMeshMpPpController.hpp"

namespace okapi {
class AsyncMotionProfileControllerBuilderMod {
public:
  /**
   * A builder that creates async motion profile controllers. Use this to build
   * an AsyncMotionProfileController or an AsyncLinearMotionProfileController.
   *
   * @param ilogger The logger this instance will log to.
   */
  explicit AsyncMotionProfileControllerBuilderMod(
      const std::shared_ptr<Logger> &ilogger = Logger::getDefaultLogger());

  /**
   * Sets the output. This must be used with
   * buildLinearMotionProfileController().
   *
   * @param ioutput The output.
   * @param idiameter The diameter of the mechanical part the motor spins.
   * @param ipair The gearset.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(const Motor &ioutput, const QLength &idiameter,
             const AbstractMotor::GearsetRatioPair &ipair);

  /**
   * Sets the output. This must be used with
   * buildLinearMotionProfileController().
   *
   * @param ioutput The output.
   * @param idiameter The diameter of the mechanical part the motor spins.
   * @param ipair The gearset.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(const MotorGroup &ioutput, const QLength &idiameter,
             const AbstractMotor::GearsetRatioPair &ipair);

  /**
   * Sets the output. This must be used with
   * buildLinearMotionProfileController().
   *
   * @param ioutput The output.
   * @param idiameter The diameter of the mechanical part the motor spins.
   * @param ipair The gearset.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(const std::shared_ptr<ControllerOutput<double>> &ioutput,
             const QLength &idiameter,
             const AbstractMotor::GearsetRatioPair &ipair);

  /**
   * Sets the output. This must be used with buildMotionProfileController().
   *
   * @param icontroller The chassis controller to use.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(ChassisController &icontroller);

  /**
   * Sets the output. This must be used with buildMotionProfileController().
   *
   * @param icontroller The chassis controller to use.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(const std::shared_ptr<ChassisController> &icontroller);

  /**
   * Sets the output. This must be used with buildMotionProfileController().
   *
   * @param imodel The chassis model to use.
   * @param iscales The chassis dimensions.
   * @param ipair The gearset.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOutput(const std::shared_ptr<ChassisModel> &imodel,
             const ChassisScales &iscales,
             const AbstractMotor::GearsetRatioPair &ipair);

  /**
   * Sets the odometry. This should be used with
   * buildMeshMpPpController().
   *
   * @param iodometry The odometry to use.
   * @param iPpConstants The constants.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withOdometry(const std::shared_ptr<Odometry> &iodometry,
               PurePursuitConstants *iPpConstants);

  /**
   * Sets the limits.
   *
   * @param ilimits The limits.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withLimits(const PathfinderLimits &ilimits);

  /**
   * Sets the TimeUtilFactory used when building the controller. The default is
   * the static TimeUtilFactory.
   *
   * @param itimeUtilFactory The TimeUtilFactory.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory);

  /**
   * Sets the logger.
   *
   * @param ilogger The logger.
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &
  withLogger(const std::shared_ptr<Logger> &ilogger);

  /**
   * Parents the internal tasks started by this builder to the current task,
   * meaning they will be deleted once the current task is deleted. The
   * `initialize` and `competition_initialize` tasks are never parented to. This
   * is the default behavior.
   *
   * Read more about this in the [builders and tasks tutorial]
   * (docs/tutorials/concepts/builders-and-tasks.md).
   *
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &parentedToCurrentTask();

  /**
   * Prevents parenting the internal tasks started by this builder to the
   * current task, meaning they will not be deleted once the current task is
   * deleted. This can cause runaway tasks, but is sometimes the desired
   * behavior (e.x., if you want to use this builder once in `autonomous` and
   * then again in `opcontrol`).
   *
   * Read more about this in the [builders and tasks tutorial]
   * (docs/tutorials/concepts/builders-and-tasks.md).
   *
   * @return An ongoing builder.
   */
  AsyncMotionProfileControllerBuilderMod &notParentedToCurrentTask();

  /**
   * Builds the AsyncLinearMotionProfileController.
   *
   * @return A fully built AsyncLinearMotionProfileController.
   */
  std::shared_ptr<AsyncLinearMotionProfileController>
  buildLinearMotionProfileController();

  /**
   * Builds the AsyncLinearMotionProfileController.
   *
   * @return A fully built AsyncLinearMotionProfileController.
   */
  std::shared_ptr<AsyncLinearMotionProfileControllerMod>
  buildLinearMotionProfileControllerMod();

  /**
   * Builds the AsyncMotionProfileController.
   *
   * @return A fully built AsyncMotionProfileController.
   */
  std::shared_ptr<AsyncMotionProfileController> buildMotionProfileController();

  /**
   * Builds the AsyncMotionProfileController.
   *
   * @return A fully built AsyncMotionProfileController.
   */
  std::shared_ptr<AsyncMeshMpPpController> buildMeshMpPpController();

private:
  std::shared_ptr<Logger> logger;

  bool hasLimits{false};
  PathfinderLimits limits;

  bool hasOutput{false};
  std::shared_ptr<ControllerOutput<double>> output;
  QLength diameter;

  bool hasModel{false};
  std::shared_ptr<ChassisModel> model;
  ChassisScales scales{{1, 1}, imev5GreenTPR};
  AbstractMotor::GearsetRatioPair pair{AbstractMotor::gearset::invalid};
  TimeUtilFactory timeUtilFactory = TimeUtilFactory();
  std::shared_ptr<Logger> controllerLogger = Logger::getDefaultLogger();

  bool hasOdometry{false};
  std::shared_ptr<Odometry> odometry;
  PurePursuitConstants *pursuitConstants;

  bool isParentedToCurrentTask{true};
};
} // namespace okapi

#endif
