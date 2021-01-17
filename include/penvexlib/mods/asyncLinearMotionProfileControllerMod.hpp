/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"

extern "C" {
#include "okapi/pathfinder/include/pathfinder.h"
}

namespace okapi {
class AsyncLinearMotionProfileControllerMod
    : public AsyncLinearMotionProfileController {
public:
  /**
   * An Async Controller which generates and follows 1D motion profiles, and
   * allows rec autom to pass it paths.
   *
   * @param itimeUtil The TimeUtil.
   * @param ilimits The default limits.
   * @param ioutput The output to write velocity targets to.
   * @param idiameter The effective diameter for whatever the motor spins.
   * @param ipair The gearset.
   * @param ilogger The logger this instance will log to.
   */
  AsyncLinearMotionProfileControllerMod(
      const TimeUtil &itimeUtil, const PathfinderLimits &ilimits,
      const std::shared_ptr<ControllerOutput<double>> &ioutput,
      const QLength &idiameter, const AbstractMotor::GearsetRatioPair &ipair,
      const std::shared_ptr<Logger> &ilogger = Logger::getDefaultLogger());

  AsyncLinearMotionProfileControllerMod(
      AsyncLinearMotionProfileControllerMod &&other) = delete;

  AsyncLinearMotionProfileControllerMod &
  operator=(AsyncLinearMotionProfileControllerMod &&other) = delete;

  ~AsyncLinearMotionProfileControllerMod() override;

  /**
   * Adds a path that is already in memory to the list of paths and transfters
   * ownership of the pointers.
   */
  void takePath(std::unique_ptr<Segment, void (*)(void *)> &itrajectory,
                int length, const std::string &ipathId);
};
} // namespace okapi
