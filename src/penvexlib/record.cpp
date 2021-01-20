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
  std::string testPath = makeFilePath(idirectory, testName + "._master.csv");

  while (fileExists(testPath)) {
    recNumber++;
    testName = "Rec" + std::to_string(recNumber);
    testPath = makeFilePath(idirectory, testName + "._master.csv");
  }
  return testName;
}

// TODO: Kill with fire as soon as possible
using namespace penvex::master;
void createBasicMasterFile(const std::string &idirectory,
                           const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ipathId + "._master.csv");
  FILE *currentFile = fopen(filePath.c_str(), "w");

  // Make sure we can open the file successfully
  if (currentFile == NULL) {
    printf("RecordIO: Couldn't open file ");
    printf("%s", filePath.c_str());
    printf(" for writing\n");
    return;
  }

  const char *baseStr = (ipathId + ".base").c_str();
  const char *intakeStr = (ipathId + ".intake").c_str();
  const char *conveyorStr = (ipathId + ".conveyor").c_str();

  // Write default file
  char buf[1024];
  sprintf(buf,
          "15,%s\nLD,B,%s\nLD,I,%s\nLD,C,%s\nMP,B,%s\nMP,I,%s\nMP,C,%s\nWIS,"
          "B\nSTOP,B\nWIS,I\nSTOP,I\nWIS,C\nSTOP,C\nRM,B,%s\nRM,I,%s\nRM,"
          "C,%s",
          idirectory.c_str(), baseStr, intakeStr, conveyorStr, baseStr,
          intakeStr, conveyorStr, baseStr, intakeStr, conveyorStr);
  fputs(buf, currentFile);

  fclose(currentFile);
}

void readMasterFile(masterFunction **&masterFunctionList, std::string &fillDir,
                    int &fillLen, const std::string &idirectory,
                    const std::string &ipathId) {
  std::string filePath = makeFilePath(idirectory, ipathId + "._master.csv");
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
  char *strData;

  fgets(line, 1024, currentFile);

  strData = strtok(line, ",\n");
  int length = std::stoi(strData, NULL);
  strData = strtok(NULL, ",\n");
  fillDir = std::string(strData);

  // NOTE: Make sure to free all the masterfunctions after use
  masterFunctionList =
      (masterFunction **)malloc(sizeof(masterFunction *) * length);

  auto readMasterFunctionSubsttId =
      [](char SubstIdChar) -> masterFunctionSubsttId {
    if (SubstIdChar == 'B')
      return B;
    if (SubstIdChar == 'I')
      return I;
    if (SubstIdChar == 'C')
      return C;
    printf("ERROR reading subst id.\n");
    return B;
  };

  int seg_n = 0;
  while (fgets(line, 1024, currentFile)) {

    strData = strtok(line, ",\n");
    masterFunctionId id;
    if (!strcmp(strData, "LD"))
      id = LD;
    if (!strcmp(strData, "RM"))
      id = RM;
    if (!strcmp(strData, "MP"))
      id = MP;
    if (!strcmp(strData, "PP"))
      id = PP;
    if (!strcmp(strData, "MPPP"))
      id = MPPP;
    if (!strcmp(strData, "TRN"))
      id = TRN;
    if (!strcmp(strData, "DRV"))
      id = DRV;
    if (!strcmp(strData, "STOP"))
      id = STOP;
    if (!strcmp(strData, "WIS"))
      id = WIS;
    if (!strcmp(strData, "WD"))
      id = WD;

    switch (id) {
    case LD:
    case RM:
    case MP: {
      masterFunctionSubstPathId *tempFunc = new masterFunctionSubstPathId();
      tempFunc->funcId = id;
      strData = strtok(NULL, ",\n");
      tempFunc->substId = readMasterFunctionSubsttId(strData[0]);
      std::string pathid(strtok(NULL, ",\n"));
      tempFunc->pathId = pathid;
      // printf("%d, %d, %s\n", (int)tempFunc->funcId, (int)tempFunc->substId,
      //        tempFunc->pathId.c_str());
      (masterFunctionList)[seg_n] = tempFunc;
    } break;

    case PP:
    case MPPP: {
      masterFunctionSubstPathIdRev *tempFunc =
          new masterFunctionSubstPathIdRev();
      tempFunc->funcId = id;
      strData = strtok(NULL, ",\n");
      tempFunc->substId = readMasterFunctionSubsttId(strData[0]);
      std::string pathid(strtok(NULL, ",\n"));
      tempFunc->pathId = pathid;
      strData = strtok(NULL, ",\n");
      if (strData[0] == 'T')
        tempFunc->rev = true;
      else
        tempFunc->rev = false;
      // printf("%d, %d, %s, %d\n", (int)tempFunc->funcId,
      // (int)tempFunc->substId,
      //        tempFunc->pathId.c_str(), (int)tempFunc->rev);
      (masterFunctionList)[seg_n] = tempFunc;
    } break;

    case TRN:
    case DRV: {
      masterFunctionDouble *tempFunc = new masterFunctionDouble();
      tempFunc->funcId = id;
      strData = strtok(NULL, ",\n");
      tempFunc->value = strtod(strData, NULL);
      // printf("%d, %f\n", (int)tempFunc->funcId, tempFunc->value);
      (masterFunctionList)[seg_n] = tempFunc;
    } break;

    case STOP:
    case WIS: {
      masterFunctionSubst *tempFunc = new masterFunctionSubst();
      tempFunc->funcId = id;
      strData = strtok(NULL, ",\n");
      tempFunc->substId = readMasterFunctionSubsttId(strData[0]);
      // printf("%d, %d\n", (int)tempFunc->funcId, (int)tempFunc->substId);
      (masterFunctionList)[seg_n] = tempFunc;
    } break;

    case WD: {
      masterFunctionDoubleXY *tempFunc = new masterFunctionDoubleXY();
      tempFunc->funcId = id;
      strData = strtok(NULL, ",\n");
      tempFunc->value = strtod(strData, NULL);
      strData = strtok(NULL, ",\n");
      tempFunc->x = strtod(strData, NULL);
      strData = strtok(NULL, ",\n");
      tempFunc->y = strtod(strData, NULL);
      // printf("%d, %f, %f, %f\n", (int)tempFunc->funcId, tempFunc->value,
      //        tempFunc->x, tempFunc->y);
      (masterFunctionList)[seg_n] = tempFunc;
    } break;

    default:
      printf("ERROR reading masterfile.\n");
      length--;
      break;
    }

    seg_n++;
  }

  fillLen = length;

  fclose(currentFile);
} // namespace penvex::record

void storeDoubles(const int length, const int numberOfValuesPerLine,
                  const double *arrays[], const std::string &idirectory,
                  const std::string &ifileName) {
  std::string filePath = makeFilePath(idirectory, ifileName + ".csv");
  FILE *currentFile = fopen(filePath.c_str(), "w");

  if (length < 1) {
    printf("RecordIO: Invalid length.");
    return;
  }

  if (numberOfValuesPerLine < 1) {
    printf("RecordIO: Invalid numberOfValuesPerLine.");
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

  const int bufLen = numberOfValuesPerLine * 25 + 1;
  for (int i = 0; i < length; i++) {
    char buf[bufLen];
    int strLen = 0;
    strLen += sprintf(buf, "%f", arrays[0][i]);
    for (int j = 1; j < numberOfValuesPerLine; j++)
      strLen += sprintf(buf + strLen, ",%f", arrays[j][i]);
    strLen += sprintf(buf + strLen, "\n");

    fputs(buf, currentFile);
  }

  fclose(currentFile);
}

void loadPath(std::shared_ptr<okapi::AsyncMeshMpPpController> &controller,
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

void loadPath(
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
