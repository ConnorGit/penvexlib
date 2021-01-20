/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "penvexlib/mods/asyncMeshMpPpController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>
#include <iostream>
#include <mutex>
#include <numeric>

namespace okapi {
AsyncMeshMpPpController::AsyncMeshMpPpController(
    const TimeUtil &itimeUtil, const PathfinderLimits &ilimits,
    PurePursuitConstants *iPpConstants,
    const std::shared_ptr<ChassisModel> &imodel, const ChassisScales &iscales,
    const AbstractMotor::GearsetRatioPair &ipair,
    const std::shared_ptr<Odometry> &iodometry,
    const std::shared_ptr<Logger> &ilogger)
    : logger(ilogger), limits(ilimits), PpConstants((*iPpConstants)),
      model(imodel), scales(iscales), pair(ipair), odom(iodometry),
      timeUtil(itimeUtil) {
  if (ipair.ratio == 0) {
    std::string msg("AsyncMeshMpPpController: The gear ratio cannot be "
                    "zero! Check if you are "
                    "using integer division.");
    LOG_ERROR(msg);
    throw std::invalid_argument(msg);
  }
}

AsyncMeshMpPpController::~AsyncMeshMpPpController() {
  dtorCalled.store(true, std::memory_order_release);

  // Free paths before deleting the task
  std::scoped_lock lock(currentPathMutex);
  paths.clear();

  delete task;
}

void AsyncMeshMpPpController::generatePath(
    std::initializer_list<PathfinderPoint> iwaypoints,
    const std::string &ipathId) {
  generatePath(iwaypoints, ipathId, limits);
}

void AsyncMeshMpPpController::generatePath(
    std::initializer_list<PathfinderPoint> iwaypoints,
    const std::string &ipathId, const PathfinderLimits &ilimits) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    LOG_WARN_S("AsyncMeshMpPpController: Not generating a path because no "
               "waypoints were given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(Waypoint{point.x.convert(meter), point.y.convert(meter),
                              point.theta.convert(radian)});
  }

  LOG_INFO_S("AsyncMeshMpPpController: Preparing trajectory");

  TrajectoryPtr candidate(new TrajectoryCandidate, [](TrajectoryCandidate *c) {
    if (c->laptr) {
      free(c->laptr);
    }

    if (c->saptr) {
      free(c->saptr);
    }

    delete c;
  });

  pathfinder_prepare(points.data(), static_cast<int>(points.size()),
                     FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.010,
                     ilimits.maxVel, ilimits.maxAccel, ilimits.maxJerk,
                     candidate.get());

  const int length = candidate->length;

  if (length < 0) {
    std::string message = "AsyncMeshMpPpController: Length was negative. " +
                          getPathErrorMessage(points, ipathId, length);

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  SegmentPtr trajectory(
      static_cast<Segment *>(malloc(length * sizeof(Segment))), free);

  if (trajectory == nullptr) {
    std::string message =
        "AsyncMeshMpPpController: Could not allocate trajectory. " +
        getPathErrorMessage(points, ipathId, length);

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  LOG_INFO_S("AsyncMeshMpPpController: Generating path");

  pathfinder_generate(candidate.get(), trajectory.get());

  SegmentPtr leftTrajectory((Segment *)malloc(sizeof(Segment) * length), free);
  SegmentPtr rightTrajectory((Segment *)malloc(sizeof(Segment) * length), free);

  if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
    std::string message =
        "AsyncMeshMpPpController: Could not allocate left and/or right "
        "trajectories. " +
        getPathErrorMessage(points, ipathId, length);

    LOG_ERROR(message);
    throw std::runtime_error(message);
  }

  LOG_INFO_S("AsyncMeshMpPpController: Modifying for tank drive");
  pathfinder_modify_tank(
      trajectory.get(), length, rightTrajectory.get(), leftTrajectory.get(),
      scales.wheelTrack.convert(meter)); // HACK: Flipped left and right

  // Free the old path before overwriting it
  forceRemovePath(ipathId);

  paths.emplace(ipathId, TrajectoryTripple{std::move(leftTrajectory),
                                           std::move(rightTrajectory),
                                           std::move(trajectory), length});

  LOG_INFO("AsyncMeshMpPpController: Completely done generating path " +
           ipathId);
  LOG_DEBUG("AsyncMeshMpPpController: Path length: " + std::to_string(length));
}

std::string AsyncMeshMpPpController::getPathErrorMessage(
    const std::vector<Waypoint> &points, const std::string &ipathId,
    int length) {
  auto pointToString = [](Waypoint point) {
    return "PathfinderPoint{x=" + std::to_string(point.x) +
           ", y=" + std::to_string(point.y) +
           ", theta=" + std::to_string(point.angle) + "}";
  };

  return "The path (id " + ipathId + ", length " + std::to_string(length) +
         ") is impossible with waypoints: " +
         std::accumulate(std::next(points.begin()), points.end(),
                         pointToString(points.at(0)),
                         [&](std::string a, Waypoint b) {
                           return a + ", " + pointToString(b);
                         });
}

bool AsyncMeshMpPpController::removePath(const std::string &ipathId) {
  if (!isDisabled() && isRunning.load(std::memory_order_acquire) &&
      getTarget() == ipathId) {
    LOG_WARN("AsyncMeshMpPpController: Attempted to remove currently "
             "running path " +
             ipathId);
    return false;
  }

  std::scoped_lock lock(currentPathMutex);

  auto oldPath = paths.find(ipathId);
  if (oldPath != paths.end()) {
    paths.erase(ipathId);
  }

  // A return value of true provides no feedback about whether the path was
  // actually removed but instead tells us that the path does not exist at this
  // moment
  return true;
}

std::vector<std::string> AsyncMeshMpPpController::getPaths() {
  std::vector<std::string> keys;

  for (const auto &path : paths) {
    keys.push_back(path.first);
  }

  return keys;
}

void AsyncMeshMpPpController::setTarget(std::string ipathId) {
  setTarget(ipathId, Mp);
}

void AsyncMeshMpPpController::setTarget(std::string ipathId,
                                        const controlMethods imethod) {
  setTarget(ipathId, imethod, false);
}

void AsyncMeshMpPpController::setTarget(std::string ipathId,
                                        const controlMethods imethod,
                                        const bool ibackwards,
                                        const bool imirrored) {
  LOG_INFO("AsyncMeshMpPpController: Set target to: " + ipathId + " (imethod=" +
           (imethod & Mp ? "Mp" : "") + (imethod & Pp ? "Pp" : "") +
           ") (ibackwards=" + std::to_string(ibackwards) +
           ", imirrored=" + std::to_string(imirrored) + ")");

  currentPath = ipathId;
  method.store(imethod, std::memory_order_release);
  direction.store(boolToSign(!ibackwards), std::memory_order_release);
  mirrored.store(imirrored, std::memory_order_release);
  isRunning.store(true, std::memory_order_release);
}

void AsyncMeshMpPpController::controllerSet(std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncMeshMpPpController::getTarget() { return currentPath; }

std::string AsyncMeshMpPpController::getProcessValue() const {
  return currentPath;
}

void AsyncMeshMpPpController::loop() {
  LOG_INFO_S("Started AsyncMeshMpPpController task.");

  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
    if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
      LOG_INFO("AsyncMeshMpPpController: Running with path: " + currentPath);

      auto path = paths.find(currentPath);
      if (path == paths.end()) {
        LOG_WARN("AsyncMeshMpPpController: Target was set to non-existent "
                 "path with name: " +
                 currentPath);
      } else {
        LOG_DEBUG("AsyncMeshMpPpController: Path length is " +
                  std::to_string(path->second.length));

        executeSinglePath(path->second, timeUtil.getRate());

        LOG_INFO_S("AsyncMeshMpPpController: Done moving");
      }

      isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("Stopped AsyncMeshMpPpController task.");
}

void AsyncMeshMpPpController::executeSinglePath(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> rate) {

  const controlMethods controllerType = method.load(std::memory_order_acquire);

  // This is garbage, but I would like to think that it is at least competent
  // garbage

  // Optomisation todo:
  // TODO: Fns are never used outside of this scope - make them lambda - if you
  // do there is no need for big stuct

  controlLoopParams params;
  params.reversed = direction.load(std::memory_order_acquire);
  params.followMirrored = mirrored.load(std::memory_order_acquire);
  params.pathLength = getPathLength(path);
  std::scoped_lock lock(purePursuitConstantsMutex);
  params.pursuitSpeed = PpConstants.pursuitSpeed;
  params.lookAheadSquare = PpConstants.lookAhead.convert(meter);
  params.lookAheadSquare *= params.lookAheadSquare;
  params.breakMoProSquare = PpConstants.breakMoPro.convert(meter);
  params.breakMoProSquare *= params.breakMoProSquare;
  params.joinDistSquare = PpConstants.joinDist.convert(meter);
  params.joinDistSquare *= params.joinDistSquare;
  purePursuitConstantsMutex.unlock();
  params.targetAPoint = false;

  controllerStepFunction currentStepFunction;

  managerFunction currentManagerFunction;

  switch (controllerType) {
  case Mp:
    currentStepFunction = &AsyncMeshMpPpController::stepMotonProfile;
    currentManagerFunction = &AsyncMeshMpPpController::manageMotionProfiling;
    break;
  case Pp:
    currentStepFunction = &AsyncMeshMpPpController::stepPurePursuit;
    currentManagerFunction = &AsyncMeshMpPpController::managePurePursuit;
    break;
  case MpPp:
    currentStepFunction = &AsyncMeshMpPpController::stepMotonProfile;
    currentManagerFunction =
        &AsyncMeshMpPpController::manageMotionProfilingMesh;
    break;
  }

  params.i = 0;
  int t = 0;
  int transitionStatus = 0;
  while (!isDisabled()) {

    transitionStatus = (this->*currentManagerFunction)(path, rate, params, t);

    if (transitionStatus == 1)
      break;
    if (transitionStatus == 3) {
      currentStepFunction = &AsyncMeshMpPpController::stepPurePursuit;
      currentManagerFunction = &AsyncMeshMpPpController::managePurePursuitMesh;
    } else if (transitionStatus == 2) {
      currentStepFunction = &AsyncMeshMpPpController::stepMotonProfile;
      currentManagerFunction =
          &AsyncMeshMpPpController::manageMotionProfilingMesh;
    }

    (this->*currentStepFunction)(path, rate, params);
  }
}

void AsyncMeshMpPpController::stepMotonProfile(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params) {
  // This mutex is used to combat an edge case of an edge case
  // if a running path is asked to be removed at the moment this loop is
  // executing
  std::scoped_lock lock(currentPathMutex);

  const auto segDT = path.left.get()[params.i].dt * second;
  const auto leftRPM =
      convertLinearToRotational(path.left.get()[params.i].velocity * mps)
          .convert(rpm);
  const auto rightRPM =
      convertLinearToRotational(path.right.get()[params.i].velocity * mps)
          .convert(rpm);

  params.rightSpeed =
      rightRPM / toUnderlyingType(pair.internalGearset) * params.reversed;
  params.leftSpeed =
      leftRPM / toUnderlyingType(pair.internalGearset) * params.reversed;
  if (params.followMirrored) {
    model->left(params.rightSpeed);
    model->right(params.leftSpeed);
  } else {
    model->left(params.leftSpeed);
    model->right(params.rightSpeed);
  }

  // Unlock before the delay to be nice to other tasks
  currentPathMutex.unlock();
  // printf("Mp\n");
  rate->delayUntil(segDT);
  params.i++;
}

void AsyncMeshMpPpController::stepPurePursuit(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params) {

  const double mirror = (params.followMirrored ? -1.0 : 1.0);

  OdomState currentPos = odom->getState();

  // This mutex is used to combat an edge case of an edge case
  // if a running path is asked to be removed at the moment this loop is
  // executing
  std::scoped_lock lock(currentPathMutex);

  /**
   * Changes i to the index of the nearest point sequentialy ahead of the
   * current index which is more than lookAhead distance away from the
   * current point. If it reaches the end of the path it will set the goal
   * to the last point.
   */
  QLength x = path.base.get()[params.i].x * meter;
  QLength y = path.base.get()[params.i].y * meter * mirror;

  double lengthToPointSquare =
      computeDistanceSquareToPoint_m({x, y}, currentPos);

  while ((!params.targetAPoint) && (params.i < params.pathLength - 1) &&
         (lengthToPointSquare <= params.lookAheadSquare)) {
    params.i++;
    x = path.base.get()[params.i].x * meter;
    y = path.base.get()[params.i].y * meter * mirror;
    lengthToPointSquare = computeDistanceSquareToPoint_m({x, y}, currentPos);
  }

  const QLength deltaTransX = (((y - currentPos.y) * cos(currentPos.theta)) -
                               ((x - currentPos.x) * sin(currentPos.theta))) *
                              params.reversed;

  double leftSaclar = 1.0;
  double rightSaclar = 1.0;

  if (deltaTransX != 0.0_m && lengthToPointSquare != 0.0) {
    const double radius =
        (lengthToPointSquare) / (2.0 * deltaTransX.convert(meter));

    // this is the wrong way to do it - split them up
    leftSaclar = (radius + ((scales.wheelTrack.convert(meter)) / 2.0));
    rightSaclar = (radius - ((scales.wheelTrack.convert(meter)) / 2.0));
  }

  // They cant both be 0
  if (fabs(leftSaclar) >= fabs(rightSaclar)) {
    params.leftSpeed = params.pursuitSpeed;
    params.rightSpeed = params.pursuitSpeed * (rightSaclar / leftSaclar);
  } else {
    params.leftSpeed = params.pursuitSpeed * (leftSaclar / rightSaclar);
    params.rightSpeed = params.pursuitSpeed;
  }

  if (params.reversed == -1) {
    model->left(-params.rightSpeed);
    model->right(-params.leftSpeed);
  } else {
    model->left(params.leftSpeed);
    model->right(params.rightSpeed);
  }

  // Unlock before the delay to be nice to other tasks
  currentPathMutex.unlock();
  // printf("Pp\n");
  rate->delayUntil(10_ms);
}

int AsyncMeshMpPpController::manageMotionProfiling(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params, int &t) {
  return (int)(params.i >= params.pathLength);
}

int AsyncMeshMpPpController::manageMotionProfilingMesh(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params, int &t) {
  if (params.pathLength - params.i >= 6) {
    const OdomState currentPos = odom->getState();
    const QLength x = path.base.get()[params.i].x * meter * params.reversed;
    const QLength y = path.base.get()[params.i].y * meter *
                      (params.followMirrored ? -1.0 : 1.0);
    const double distSquare =
        (computeDistanceSquareToPoint_m({x, y}, currentPos));
    if (distSquare >= params.breakMoProSquare)
      return 3;
  }
  return (int)(params.i >= params.pathLength);
}

int AsyncMeshMpPpController::managePurePursuit(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params, int &t) {
  if (params.i == params.pathLength - 1) {
    params.targetAPoint = true;
    t++;
    const OdomState currentPos = odom->getState();
    const QLength x = path.base.get()[params.i].x * meter * params.reversed;
    const QLength y = path.base.get()[params.i].y * meter *
                      (params.followMirrored ? -1.0 : 1.0);
    const double distSquare =
        (computeDistanceSquareToPoint_m({x, y}, currentPos));

    std::scoped_lock lock(purePursuitConstantsMutex);
    params.pursuitSpeed = PpConstants.pursuitSpeed *
                          std::sqrt(distSquare / params.lookAheadSquare);
    if (params.pursuitSpeed > PpConstants.pursuitSpeed) {
      params.pursuitSpeed = PpConstants.pursuitSpeed;
    }
    printf("%f\n", params.pursuitSpeed);
    purePursuitConstantsMutex.unlock();
    return (int)((distSquare <= params.joinDistSquare) ||
                 (t >= 50)); // TODO: magic number2
  }
  if (params.targetAPoint) {
    params.targetAPoint = false;
    std::scoped_lock lock(purePursuitConstantsMutex);
    params.pursuitSpeed = PpConstants.pursuitSpeed;
    purePursuitConstantsMutex.unlock();
    t = 0;
  }
  return 0;
}

int AsyncMeshMpPpController::managePurePursuitMesh(
    const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    controlLoopParams &params, int &t) {
  const double leftVelocity = path.left.get()[params.i].velocity;
  const double rightVelocity = path.right.get()[params.i].velocity;
  // If curvature matches or end of path
  if ((fabs((params.leftSpeed * path.right.get()[params.i].velocity) -
            (path.left.get()[params.i].velocity * params.rightSpeed)) <=
       0.005) ||
      (params.i == params.pathLength - 1)) {
    params.targetAPoint = true;

    const OdomState currentPos = odom->getState();
    const QLength x = path.base.get()[params.i].x * meter * params.reversed;
    const QLength y = path.base.get()[params.i].y * meter *
                      (params.followMirrored ? -1.0 : 1.0);
    const double distSquare =
        (computeDistanceSquareToPoint_m({x, y}, currentPos));

    double targetSpeed;

    if (params.i == params.pathLength - 1) {
      t++;
      targetSpeed = 0.0;
    } else {
      if (fabs(leftVelocity) >= fabs(rightVelocity))
        targetSpeed =
            convertLinearToRotational(fabs(leftVelocity) * mps).convert(rpm) /
            toUnderlyingType(pair.internalGearset) * params.reversed;
      else
        targetSpeed =
            convertLinearToRotational(fabs(rightVelocity) * mps).convert(rpm) /
            toUnderlyingType(pair.internalGearset) * params.reversed;
    }

    const double percentChange = std::sqrt(distSquare / params.lookAheadSquare);

    std::scoped_lock lock(purePursuitConstantsMutex);
    params.pursuitSpeed = PpConstants.pursuitSpeed * percentChange +
                          targetSpeed * (1.0 - percentChange);
    if (PpConstants.pursuitSpeed >= targetSpeed) {
      if (params.pursuitSpeed > PpConstants.pursuitSpeed) {
        params.pursuitSpeed = PpConstants.pursuitSpeed;
      }
    } else if (params.pursuitSpeed > targetSpeed) {
      params.pursuitSpeed = targetSpeed;
    }
    purePursuitConstantsMutex.unlock();

    // printf("%f, %f\n", params.pursuitSpeed, targetSpeed);

    if (params.i == params.pathLength - 1)
      return (int)((distSquare <= params.joinDistSquare) ||
                   (t >= 150)); // TODO: magic number2
    else
      return (2 * (int)(distSquare <= params.joinDistSquare));
  }
  if (params.targetAPoint) {
    params.targetAPoint = false;
    std::scoped_lock lock(purePursuitConstantsMutex);
    params.pursuitSpeed = PpConstants.pursuitSpeed;
    purePursuitConstantsMutex.unlock();
    t = 0;
  }
  return 0;
}

int AsyncMeshMpPpController::getPathLength(const TrajectoryTripple &path) {
  std::scoped_lock lock(currentPathMutex);
  return path.length;
}

QAngularSpeed
AsyncMeshMpPpController::convertLinearToRotational(QSpeed linear) const {
  return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * pair.ratio;
}

double AsyncMeshMpPpController::computeDistanceSquareToPoint_m(
    const Point &ipoint, const OdomState &istate) {
  const double xDiff = (ipoint.x - istate.x).convert(meter);
  const double yDiff = (ipoint.y - istate.y).convert(meter);
  return ((xDiff * xDiff) + (yDiff * yDiff));
}

void AsyncMeshMpPpController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncMeshMpPpController *>(context)->loop();
  }
}

void AsyncMeshMpPpController::waitUntilSettled() {
  LOG_INFO_S("AsyncMeshMpPpController: Waiting to settle");

  auto rate = timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  LOG_INFO_S("AsyncMeshMpPpController: Done waiting to settle");
}

void AsyncMeshMpPpController::moveTo(
    std::initializer_list<PathfinderPoint> iwaypoints, controlMethods imethod,
    bool ibackwards, bool imirrored) {
  moveTo(iwaypoints, limits, imethod, ibackwards, imirrored);
}

void AsyncMeshMpPpController::moveTo(
    std::initializer_list<PathfinderPoint> iwaypoints,
    const PathfinderLimits &ilimits, const controlMethods imethod,
    const bool ibackwards, const bool imirrored) {
  static int moveToCount = 0;
  std::string name = "__moveTo" + std::to_string(moveToCount++);
  generatePath(iwaypoints, name, ilimits);
  setTarget(name, imethod, ibackwards, imirrored);
  waitUntilSettled();
  forceRemovePath(name);
}

PathfinderPoint AsyncMeshMpPpController::getError() const {
  return PathfinderPoint{0_m, 0_m, 0_deg};
}

bool AsyncMeshMpPpController::isSettled() {
  return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncMeshMpPpController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  LOG_INFO_S("AsyncMeshMpPpController: Waiting to reset");

  auto rate = timeUtil.getRate();
  while (isRunning.load(std::memory_order_acquire)) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncMeshMpPpController::flipDisable() {
  flipDisable(!disabled.load(std::memory_order_acquire));
}

void AsyncMeshMpPpController::flipDisable(const bool iisDisabled) {
  LOG_INFO("AsyncMeshMpPpController: flipDisable " +
           std::to_string(iisDisabled));
  disabled.store(iisDisabled, std::memory_order_release);
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncMeshMpPpController::isDisabled() const {
  return disabled.load(std::memory_order_acquire);
}

void AsyncMeshMpPpController::tarePosition() {}

void AsyncMeshMpPpController::setMaxVelocity(std::int32_t) {}

void AsyncMeshMpPpController::setPurePursuitConstants(
    const PurePursuitConstants &iPpConstants) {
  std::scoped_lock lock(purePursuitConstantsMutex);
  PpConstants = iPpConstants;
  purePursuitConstantsMutex.unlock();
}

void AsyncMeshMpPpController::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this, "AsyncMeshMpPpController");
  }
}

CrossplatformThread *AsyncMeshMpPpController::getThread() const { return task; }

void AsyncMeshMpPpController::storePath(const std::string &idirectory,
                                        const std::string &ipathId) {
  std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
  std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
  std::string baseFilePath = makeFilePath(idirectory, ipathId + ".base.csv");
  FILE *leftPathFile = fopen(leftFilePath.c_str(), "w");
  FILE *rightPathFile = fopen(rightFilePath.c_str(), "w");
  FILE *basePathFile = fopen(baseFilePath.c_str(), "w");

  // Make sure we can open the file successfully
  if (leftPathFile == NULL || rightPathFile == NULL || basePathFile == NULL) {
    LOG_WARN("AsyncMeshMpPpController: Couldn't open either file " +
             leftFilePath + ", " + rightFilePath + ", or " + baseFilePath +
             " for writing");
    if (leftPathFile != NULL) {
      fclose(leftPathFile);
    }
    if (rightPathFile != NULL) {
      fclose(rightPathFile);
    }
    if (basePathFile != NULL) {
      fclose(basePathFile);
    }
    return;
  }

  internalStorePath(leftPathFile, rightPathFile, basePathFile, ipathId);

  fclose(leftPathFile);
  fclose(rightPathFile);
  fclose(basePathFile);
}

void AsyncMeshMpPpController::loadPath(const std::string &idirectory,
                                       const std::string &ipathId) {
  std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
  std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
  std::string baseFilePath = makeFilePath(idirectory, ipathId + ".base.csv");
  FILE *leftPathFile = fopen(leftFilePath.c_str(), "r");
  FILE *rightPathFile = fopen(rightFilePath.c_str(), "r");
  FILE *basePathFile = fopen(baseFilePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (leftPathFile == NULL || rightPathFile == NULL || basePathFile == NULL) {
    LOG_WARN("AsyncMeshMpPpController: Couldn't open either file " +
             leftFilePath + ", " + rightFilePath + ", or " + baseFilePath +
             " for reading");
    if (leftPathFile != NULL) {
      fclose(leftPathFile);
    }
    if (rightPathFile != NULL) {
      fclose(rightPathFile);
    }
    if (basePathFile != NULL) {
      fclose(basePathFile);
    }
    return;
  }

  internalLoadPath(leftPathFile, rightPathFile, basePathFile, ipathId);

  fclose(leftPathFile);
  fclose(rightPathFile);
  fclose(basePathFile);
}

void AsyncMeshMpPpController::internalStorePath(FILE *leftPathFile,
                                                FILE *rightPathFile,
                                                FILE *basePathFile,
                                                const std::string &ipathId) {
  auto pathData = this->paths.find(ipathId);

  // Make sure path exists
  if (pathData == paths.end()) {
    LOG_WARN("AsyncMeshMpPpController: Controller was asked to serialize "
             "non-existent path " +
             ipathId);
    // Do nothing- can't serialize nonexistent path
  } else {
    int len = pathData->second.length;

    // Serialize paths
    pathfinder_serialize_csv(leftPathFile, pathData->second.left.get(), len);
    pathfinder_serialize_csv(rightPathFile, pathData->second.right.get(), len);
    pathfinder_serialize_csv(basePathFile, pathData->second.base.get(), len);
  }
}

void AsyncMeshMpPpController::internalLoadPath(FILE *leftPathFile,
                                               FILE *rightPathFile,
                                               FILE *basePathFile,
                                               const std::string &ipathId) {
  // TODO: Speed up loading paths from memory either here or create an optional
  // faster version which only loads v and x,y and donst cound lines as slowle

  // Count lines in file, remove one for headers
  int count = 0;
  for (int c = getc(leftPathFile); c != EOF; c = getc(leftPathFile)) {
    if (c == '\n') {
      ++count;
    }
  }
  --count;
  rewind(leftPathFile);

  // Allocate memory
  SegmentPtr leftTrajectory((Segment *)malloc(sizeof(Segment) * count), free);
  SegmentPtr rightTrajectory((Segment *)malloc(sizeof(Segment) * count), free);
  SegmentPtr baseTrajectory((Segment *)malloc(sizeof(Segment) * count), free);

  pathfinder_deserialize_csv(leftPathFile, leftTrajectory.get());
  pathfinder_deserialize_csv(rightPathFile, rightTrajectory.get());
  pathfinder_deserialize_csv(basePathFile, baseTrajectory.get());

  // Remove the old path if it exists
  forceRemovePath(ipathId);
  paths.emplace(ipathId, TrajectoryTripple{std::move(leftTrajectory),
                                           std::move(rightTrajectory),
                                           std::move(baseTrajectory), count});
}

std::string AsyncMeshMpPpController::makeFilePath(const std::string &directory,
                                                  const std::string &filename) {
  std::string path(directory);

  // Checks first substring
  if (path.rfind("/usd", 0) == std::string::npos) {
    if (path.rfind("usd", 0) != std::string::npos) {
      // There's a usd, but no beginning slash
      path.insert(0, "/"); // We just need a slash
    } else {               // There's nothing at all
      if (path.front() == '/') {
        // Don't double up on slashes
        path.insert(0, "/usd");
      } else {
        path.insert(0, "/usd/");
      }
    }
  }

  // Add trailing slash if there isn't one
  if (path.back() != '/') {
    path.append("/");
  }
  std::string filenameCopy(filename);
  // Remove restricted characters from filename
  static const std::string illegalChars = "\\/:?*\"<>|";
  for (auto it = filenameCopy.begin(); it < filenameCopy.end(); it++) {
    if (illegalChars.rfind(*it) != std::string::npos) {
      it = filenameCopy.erase(it);
    }
  }

  path.append(filenameCopy);

  return path;
}

void AsyncMeshMpPpController::forceRemovePath(const std::string &ipathId) {
  if (!removePath(ipathId)) {
    LOG_WARN("AsyncMeshMpPpController: Disabling controller to remove path " +
             ipathId);
    flipDisable(true);
    removePath(ipathId);
  }
}

void AsyncMeshMpPpController::takePath(
    std::unique_ptr<Segment, void (*)(void *)> &ileftTrajectory,
    std::unique_ptr<Segment, void (*)(void *)> &irightTrajectory,
    std::unique_ptr<Segment, void (*)(void *)> &ibaseTrajectory, int length,
    const std::string &ipathId) {

  // Remove the old path if it exists
  forceRemovePath(ipathId);
  paths.emplace(ipathId, TrajectoryTripple{std::move(ileftTrajectory),
                                           std::move(irightTrajectory),
                                           std::move(ibaseTrajectory), length});
}
} // namespace okapi
