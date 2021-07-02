/*
 * This contains helper functions and file IO for recording data.
 */

#include "main.h"
#include <iostream>
#include <string.h>

namespace penvex::record {

std::string makeFilePath(const std::string &directory,
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

bool fileExists(const std::string &ifileName) {
  if (FILE *file = fopen(ifileName.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

std::string getNewRecID(const std::string &idirectory) {
  int recNumber = 1;
  std::string testName = "Rec" + std::to_string(recNumber);
  std::string testPath = makeFilePath(idirectory, testName + "._master.txt");

  while (fileExists(testPath)) {
    recNumber++;
    testName = "Rec" + std::to_string(recNumber);
    testPath = makeFilePath(idirectory, testName + "._master.txt");
  }
  return testName;
}

void createBasicMasterFile(const std::string &idirectory,
                           const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ipathId + "._master.txt");
  FILE *currentFile = fopen(filePath.c_str(), "w");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for writing\n");
    return;
  }

  // Write default file
  char buf[1024];
  sprintf(buf, "Recording: %s\nRec Dir: %s", ipathId.c_str(),
          idirectory.c_str());
  fputs(buf, currentFile);

  fclose(currentFile);
}

// Not inplemented currently
void readMasterFile(const std::string &idirectory, const std::string &ipathId) {
}

void storeDoubles(const int length, const int numberOfBytesPerFrame,
                  const double *arrays[], const std::string &idirectory,
                  const std::string &ifileName) {
  std::string filePath = makeFilePath(idirectory, ifileName);
  FILE *currentFile = fopen(filePath.c_str(), "w");

  if (length < 1) {
    printf("RecordIO: Invalid length writing %s.", ifileName.c_str());
    return;
  }

  // 8 bytes in a double
  if ((numberOfBytesPerFrame < 0) || (numberOfBytesPerFrame % 8 != 0)) {
    printf("RecordIO: Invalid numberOfBytesPerFrame writing %s.",
           ifileName.c_str());
    return;
  }

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for writing\n");
    return;
  }

  char buf_1[4];
  intToBytes(length, buf_1);
  fwrite(buf_1, 1, 4, currentFile);

  char buf[8];
  int i, j;
  for (i = 0; i < length; i++) {
    for (j = 0; j < (numberOfBytesPerFrame / 8); j++) {
      doubleToBytes(arrays[j][i], buf);
      fwrite(buf, 1, 8, currentFile);
    }
  }

  fclose(currentFile);
}

void loadPath(std::shared_ptr<okapi::AsyncMeshMpPpController> &controller,
              const std::string &idirectory, const std::string &ifileName,
              const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ifileName);
  FILE *currentFile = fopen(filePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for reading\n");
    return;
  }

  char buf_1[4];
  fread(buf_1, 1, 4, currentFile);
  int length = bytesToInt(buf_1);

  std::unique_ptr<Segment, void (*)(void *)> leftTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);
  std::unique_ptr<Segment, void (*)(void *)> rightTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);
  std::unique_ptr<Segment, void (*)(void *)> baseTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);

  char buf[8];

  int i;
  for (i = 0; i < length; i++) {
    fread(buf, 1, 8, currentFile);
    double dt = bytesToDouble(buf);

    fread(buf, 1, 8, currentFile);
    double x = bytesToDouble(buf);

    fread(buf, 1, 8, currentFile);
    double y = bytesToDouble(buf);

    fread(buf, 1, 8, currentFile);
    double velL = bytesToDouble(buf);

    fread(buf, 1, 8, currentFile);
    double velR = bytesToDouble(buf);

    Segment baseLSeg = {dt, 0.0, 0.0, 0.0, velL, 0.0, 0.0, 0.0};
    (leftTrajectory.get())[i] = baseLSeg;
    Segment baseRSeg = {dt, 0.0, 0.0, 0.0, velR, 0.0, 0.0, 0.0};
    (rightTrajectory.get())[i] = baseRSeg;
    Segment baseSeg = {dt, x, y, 0.0, (velR + velL) * 0.5, 0.0, 0.0, 0.0};
    (baseTrajectory.get())[i] = baseSeg;
  }

  fclose(currentFile);

  // Give path to controller
  controller->takePath(leftTrajectory, rightTrajectory, baseTrajectory, length,
                       ipathId);
}

void loadPath(
    std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod> &controller,
    const std::string &idirectory, const std::string &ifileName,
    const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ifileName);
  FILE *currentFile = fopen(filePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for reading\n");
    return;
  }

  char buf_1[4];
  fread(buf_1, 1, 4, currentFile);
  int length = bytesToInt(buf_1);

  std::unique_ptr<Segment, void (*)(void *)> trajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);

  char buf[8];

  int i;
  for (i = 0; i < length; i++) {
    fread(buf, 1, 8, currentFile);
    double dt = bytesToDouble(buf);

    fread(buf, 1, 8, currentFile);
    double vel = bytesToDouble(buf);

    Segment seg = {dt, 0.0, 0.0, 0.0, vel, 0.0, 0.0, 0.0};
    (trajectory.get())[i] = seg;
  }

  fclose(currentFile);

  // Give path to controller
  controller->takePath(trajectory, length, ipathId);
}

// csv reads and Writes

void storeDoubles_csv(const int length, const int numberOfBytesPerFrame,
                      const double *arrays[], const std::string &idirectory,
                      const std::string &ifileName) {
  std::string filePath = makeFilePath(idirectory, ifileName + ".csv");
  FILE *currentFile = fopen(filePath.c_str(), "w");

  if (length < 1) {
    printf("RecordIO: Invalid length writing %s.", ifileName.c_str());
    return;
  }

  // 8 bytes in a double
  if ((numberOfBytesPerFrame < 0) || (numberOfBytesPerFrame % 8 != 0)) {
    printf("RecordIO: Invalid numberOfBytesPerFrame writing %s.",
           ifileName.c_str());
    return;
  }

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for writing\n");
    return;
  }

  char buf1[50];
  sprintf(buf1, "%d\n", length);
  fputs(buf1, currentFile);

  const int bufLen = (numberOfBytesPerFrame / 8) * 25 + 1;
  for (int i = 0; i < length; i++) {
    char buf[bufLen];
    int strLen = 0;
    strLen += sprintf(buf, "%f", arrays[0][i]);
    for (int j = 1; j < (numberOfBytesPerFrame / 8); j++)
      strLen += sprintf(buf + strLen, ",%f", arrays[j][i]);
    strLen += sprintf(buf + strLen, "\n");

    fputs(buf, currentFile);
  }

  fclose(currentFile);
}

void loadPath_csv(std::shared_ptr<okapi::AsyncMeshMpPpController> &controller,
                  const std::string &idirectory, const std::string &ifileName,
                  const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ifileName + ".csv");
  FILE *currentFile = fopen(filePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for reading\n");
    return;
  }

  // NOTE: Not threadsafe

  char line[1024];
  char *number;

  fgets(line, 1024, currentFile);

  number = strtok(line, ",");
  int length = std::stoi(number, NULL);

  std::unique_ptr<Segment, void (*)(void *)> leftTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);
  std::unique_ptr<Segment, void (*)(void *)> rightTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);
  std::unique_ptr<Segment, void (*)(void *)> baseTrajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);

  int seg_n = 0;
  while (fgets(line, 1024, currentFile)) {

    number = strtok(line, ",");
    double dt = strtod(number, NULL);
    // printf("%f,", dt);
    number = strtok(NULL, ",");
    double x = strtod(number, NULL);
    // printf("%f,", x);
    number = strtok(NULL, ",");
    double y = strtod(number, NULL);
    // printf("%f,", y);
    number = strtok(NULL, ",");
    double velL = strtod(number, NULL);
    // printf("%f,", velL);
    number = strtok(NULL, ",");
    double velR = strtod(number, NULL);
    // printf("%f\n", velR);

    Segment baseLSeg = {dt, 0.0, 0.0, 0.0, velL, 0.0, 0.0, 0.0};
    (leftTrajectory.get())[seg_n] = baseLSeg;
    Segment baseRSeg = {dt, 0.0, 0.0, 0.0, velR, 0.0, 0.0, 0.0};
    (rightTrajectory.get())[seg_n] = baseRSeg;
    Segment baseSeg = {dt, x, y, 0.0, (velR + velL) * 0.5, 0.0, 0.0, 0.0};
    (baseTrajectory.get())[seg_n] = baseSeg;

    seg_n++;
  }

  fclose(currentFile);

  // Give path to controller
  controller->takePath(leftTrajectory, rightTrajectory, baseTrajectory, length,
                       ipathId);
}

void loadPath_csv(
    std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod> &controller,
    const std::string &idirectory, const std::string &ifileName,
    const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ifileName + ".csv");
  FILE *currentFile = fopen(filePath.c_str(), "r");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for reading\n");
    return;
  }

  // NOTE: Not threadsafe

  char line[1024];
  char *number;

  fgets(line, 1024, currentFile);

  number = strtok(line, ",");
  int length = std::stoi(number, NULL);

  std::unique_ptr<Segment, void (*)(void *)> trajectory(
      (Segment *)malloc(sizeof(Segment) * length), free);

  int seg_n = 0;
  while (fgets(line, 1024, currentFile)) {

    // printf("%d,", seg_n + 2);
    number = strtok(line, ",");
    double dt = strtod(number, NULL);
    // printf("%f,", dt);
    number = strtok(NULL, ",");
    double vel = strtod(number, NULL);
    // printf("%f\n", vel);

    Segment seg = {dt, 0.0, 0.0, 0.0, vel, 0.0, 0.0, 0.0};
    (trajectory.get())[seg_n] = seg;

    seg_n++;
  }

  fclose(currentFile);

  // Give path to controller
  controller->takePath(trajectory, length, ipathId);
}
} // namespace penvex::record
