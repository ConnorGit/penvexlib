/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "penvexlib/mods/asyncLinearMotionProfileControllerMod.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <mutex>
#include <numeric>

namespace okapi {
AsyncLinearMotionProfileControllerMod::AsyncLinearMotionProfileControllerMod(
    const TimeUtil &itimeUtil, const PathfinderLimits &ilimits,
    const std::shared_ptr<ControllerOutput<double>> &ioutput,
    const QLength &idiameter, const AbstractMotor::GearsetRatioPair &ipair,
    const std::shared_ptr<Logger> &ilogger)
    : AsyncLinearMotionProfileController(itimeUtil, ilimits, ioutput, idiameter,
                                         ipair, ilogger) {}

AsyncLinearMotionProfileControllerMod::
    ~AsyncLinearMotionProfileControllerMod() {
  dtorCalled.store(true, std::memory_order_release);

  // Free paths before deleting the task
  std::scoped_lock lock(currentPathMutex);
  paths.clear();

  delete task;
}

void AsyncLinearMotionProfileControllerMod::takePath(
    std::unique_ptr<Segment, void (*)(void *)> &itrajectory, int length,
    const std::string &ipathId) {

  // Remove the old path if it exists
  forceRemovePath(ipathId);
  paths.emplace(ipathId, TrajectoryPair{std::move(itrajectory), length});
}

} // namespace okapi
