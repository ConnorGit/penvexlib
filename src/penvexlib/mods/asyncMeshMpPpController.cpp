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
    const std::shared_ptr<ChassisModel> &imodel, const ChassisScales &iscales,
    const AbstractMotor::GearsetRatioPair &ipair,
    const std::shared_ptr<Odometry> &iodometry,
    const std::shared_ptr<Logger> &ilogger)
    : logger(ilogger), limits(ilimits), model(imodel), scales(iscales),
      pair(ipair), odom(iodometry), timeUtil(itimeUtil) {
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
  pathfinder_modify_tank(trajectory.get(), length, leftTrajectory.get(),
                         rightTrajectory.get(),
                         scales.wheelTrack.convert(meter));

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
  setTarget(ipathId, false);
}

void AsyncMeshMpPpController::setTarget(std::string ipathId,
                                        const bool ibackwards,
                                        const bool imirrored) {
  LOG_INFO("AsyncMeshMpPpController: Set target to: " + ipathId +
           " (ibackwards=" + std::to_string(ibackwards) +
           ", imirrored=" + std::to_string(imirrored) + ")");

  currentPath = ipathId;
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
  const int reversed = direction.load(std::memory_order_acquire);
  const bool followMirrored = mirrored.load(std::memory_order_acquire);
  const int pathLength = getPathLength(path);

  int i = 0;
  while (!isDisabled()) {
    if (i >= pathLength)
      break;

    stepMotonProfile(i, path, rate, reversed, followMirrored, pathLength);
  }
}

void AsyncMeshMpPpController::stepMotonProfile(
    int &i, const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    const int reversed, const bool followMirrored, const int pathLength) {
  // This mutex is used to combat an edge case of an edge case
  // if a running path is asked to be removed at the moment this loop is
  // executing
  std::scoped_lock lock(currentPathMutex);

  const auto segDT = path.left.get()[i].dt * second;
  const auto leftRPM =
      convertLinearToRotational(path.left.get()[i].velocity * mps).convert(rpm);
  const auto rightRPM =
      convertLinearToRotational(path.right.get()[i].velocity * mps)
          .convert(rpm);

  const double rightSpeed =
      rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
  const double leftSpeed =
      leftRPM / toUnderlyingType(pair.internalGearset) * reversed;
  if (followMirrored) {
    model->left(rightSpeed);
    model->right(leftSpeed);
  } else {
    model->left(leftSpeed);
    model->right(rightSpeed);
  }

  // Unlock before the delay to be nice to other tasks
  currentPathMutex.unlock();

  rate->delayUntil(segDT);
  i++;
}

void AsyncMeshMpPpController::stepPurePursuit(
    int &i, const TrajectoryTripple &path, std::unique_ptr<AbstractRate> &rate,
    const int reversed, const bool followMirrored, const int pathLength) {

  OdomState currentPos = odom->getState();

  // This mutex is used to combat an edge case of an edge case
  // if a running path is asked to be removed at the moment this loop is
  // executing
  std::scoped_lock lock(currentPathMutex);

  QLength lookAhead = 10_in;

  /**
   * Changes i to the index of the nearest point sequentialy ahead of the
   * current index which is more than lookAhead distance away from the current
   * point.
   * If it reaches the end of the path it will set the goal to the last point.
   */
  QLength x = path.base.get()[i].x * meter;
  QLength y = path.base.get()[i].y * meter;
  while ((i != pathLength) &&
         (OdomMath::computeDistanceToPoint({x, y}, currentPos) <= lookAhead)) {
    i++;
    x = path.base.get()[i].x * meter;
    y = path.base.get()[i].y * meter;
  }

  const QLength lengthToPoint =
      OdomMath::computeDistanceToPoint({x, y}, currentPos);

  const QLength deltaTransX = ((y - currentPos.y) * cos(currentPos.theta)) +
                              ((x - currentPos.x) * sin(currentPos.theta));

  if (deltaTransX != 0.0_m && lengthToPoint != 0.0_m) {
    const QLength radius =
        (lengthToPoint * lengthToPoint) / (2.0 * deltaTransX);

    // this is the wrong way to do it - split them up
    const auto LratioToR = (radius + ((scales.wheelTrack) / 2.0)) /
                           (radius - ((scales.wheelTrack) / 2.0));

  } else
    const const auto LratioToR = 1.0;

  const auto segDT = path.left.get()[i].dt * second;
  const auto leftRPM =
      convertLinearToRotational(path.left.get()[i].velocity * mps).convert(rpm);
  const auto rightRPM =
      convertLinearToRotational(path.right.get()[i].velocity * mps)
          .convert(rpm);

  const double rightSpeed =
      rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
  const double leftSpeed =
      leftRPM / toUnderlyingType(pair.internalGearset) * reversed;
  if (followMirrored) {
    model->left(rightSpeed);
    model->right(leftSpeed);
  } else {
    model->left(leftSpeed);
    model->right(rightSpeed);
  }

  // Unlock before the delay to be nice to other tasks
  currentPathMutex.unlock();

  rate->delayUntil(segDT);
  i++;
}

int AsyncMeshMpPpController::getPathLength(const TrajectoryTripple &path) {
  std::scoped_lock lock(currentPathMutex);
  return path.length;
}

QAngularSpeed
AsyncMeshMpPpController::convertLinearToRotational(QSpeed linear) const {
  return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * pair.ratio;
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
    std::initializer_list<PathfinderPoint> iwaypoints, bool ibackwards,
    bool imirrored) {
  moveTo(iwaypoints, limits, ibackwards, imirrored);
}

void AsyncMeshMpPpController::moveTo(
    std::initializer_list<PathfinderPoint> iwaypoints,
    const PathfinderLimits &ilimits, const bool ibackwards,
    const bool imirrored) {
  static int moveToCount = 0;
  std::string name = "__moveTo" + std::to_string(moveToCount++);
  generatePath(iwaypoints, name, ilimits);
  setTarget(name, ibackwards, imirrored);
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
} // namespace okapi
