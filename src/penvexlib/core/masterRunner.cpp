// #include "main.h"
//
// namespace penvex::master {
//
// void runMasterFile(const std::string &idirectory, const std::string &ipathId)
// {
//   masterFunction **masterFunctionList = nullptr;
//   int len;
//   std::string dirName;
//   penvex::record::readMasterFile(masterFunctionList, dirName, len,
//   idirectory,
//                                  ipathId);
//
//   if (masterFunctionList == nullptr)
//     return;
//
//   // resetData();
//
//   // TODO: this is bad for a lot of reasons but one is that inheritance is
//   not
//   // properly implemented
//
//   for (int i = 0; i < len; i++) {
//     switch (masterFunctionList[i]->funcId) {
//
//     case LD:
//       switch (((masterFunctionSubstPathId *)masterFunctionList[i])->substId)
//       { case B:
//         penvex::record::loadPath(
//             profileBaseController, dirName,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       case I:
//         penvex::record::loadPath(
//             profileIntakeController, dirName,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       case C:
//         penvex::record::loadPath(
//             profileConveyorController, dirName,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId,
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       }
//       break;
//
//     case RM:
//       switch (((masterFunctionSubstPathId *)masterFunctionList[i])->substId)
//       { case B:
//         profileBaseController->removePath(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       case I:
//         profileIntakeController->removePath(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       case C:
//         profileConveyorController->removePath(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         break;
//       }
//       break;
//
//     case MP:
//       switch (((masterFunctionSubstPathId *)masterFunctionList[i])->substId)
//       { case B:
//         profileBaseController->setTarget(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId,
//             okapi::AsyncMeshMpPpController::Mp, false, false);
//         profileBaseController->flipDisable(false);
//         break;
//       case I:
//         profileIntakeController->setTarget(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         profileIntakeController->flipDisable(false);
//         break;
//       case C:
//         profileConveyorController->setTarget(
//             ((masterFunctionSubstPathId *)masterFunctionList[i])->pathId);
//         profileConveyorController->flipDisable(false);
//         break;
//       }
//       break;
//
//     case PP:
//       profileBaseController->setTarget(
//           ((masterFunctionSubstPathIdRev *)masterFunctionList[i])->pathId,
//           okapi::AsyncMeshMpPpController::Pp,
//           ((masterFunctionSubstPathIdRev *)masterFunctionList[i])->rev,
//           false);
//       profileBaseController->flipDisable(false);
//       break;
//
//     case MPPP:
//       profileBaseController->setTarget(
//           ((masterFunctionSubstPathIdRev *)masterFunctionList[i])->pathId,
//           okapi::AsyncMeshMpPpController::MpPp,
//           ((masterFunctionSubstPathIdRev *)masterFunctionList[i])->rev,
//           false);
//       profileBaseController->flipDisable(false);
//       break;
//
//     case TRN:
//       base->turnToAngle(
//           (((masterFunctionDouble *)masterFunctionList[i])->value) *
//           okapi::degree);
//       break;
//
//     case DRV:
//       base->moveDistance(
//           (((masterFunctionDouble *)masterFunctionList[i])->value) *
//           okapi::inch);
//       break;
//
//     case STOP:
//       switch (((masterFunctionSubst *)masterFunctionList[i])->substId) {
//       case B:
//         profileBaseController->flipDisable(true);
//         base->stop();
//         break;
//       case I:
//         profileIntakeController->flipDisable(true);
//         intake->moveVoltage(0);
//         break;
//       case C:
//         profileConveyorController->flipDisable(true);
//         conveyor->moveVoltage(0);
//         break;
//       }
//       break;
//
//     case WIS:
//       switch (((masterFunctionSubst *)masterFunctionList[i])->substId) {
//       case B:
//         profileBaseController->waitUntilSettled();
//         break;
//       case I:
//         profileIntakeController->waitUntilSettled();
//         break;
//       case C:
//         profileConveyorController->waitUntilSettled();
//         break;
//       }
//       break;
//
//     case WD: {
//       double breakDistSquare =
//           ((masterFunctionDoubleXY *)masterFunctionList[i])->value;
//       breakDistSquare *= breakDistSquare;
//       okapi::OdomState currentPos = base->getOdometry()->getState();
//
//       const double xTarg = ((masterFunctionDoubleXY
//       *)masterFunctionList[i])->x; const double yTarg =
//       ((masterFunctionDoubleXY *)masterFunctionList[i])->y; double xDiff =
//       (xTarg - currentPos.x.convert(okapi::inch)); double yDiff = (yTarg -
//       currentPos.y.convert(okapi::inch));
//
//       while (((xDiff * xDiff) + (yDiff * yDiff)) >= breakDistSquare) {
//         currentPos = base->getOdometry()->getState();
//         xDiff = (xTarg - currentPos.x.convert(okapi::inch));
//         yDiff = (yTarg - currentPos.y.convert(okapi::inch));
//         pros::Task::delay(40);
//       }
//     } break;
//
//     case DTP:
//       base->driveToPoint(
//           okapi::Point{((((masterFunctionDoubleXY
//           *)masterFunctionList[i])->x) *
//                         okapi::meter),
//                        ((((masterFunctionDoubleXY
//                        *)masterFunctionList[i])->y) *
//                         okapi::meter)},
//           (((masterFunctionDoubleXY *)masterFunctionList[i])->value ==
//           -1.0));
//       break;
//
//     default:
//       break;
//     }
//
//     free(masterFunctionList[i]);
//   }
//   free(masterFunctionList);
// }
// } // namespace penvex::master
