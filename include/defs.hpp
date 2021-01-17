
#ifndef _DEFS_HPP_
#define _DEFS_HPP_

#include "okapi/api.hpp"
#include "penvexlib/api.hpp"
#include <memory>

/**
 * Which subsystems the macro involves - to avoid competition and allow for
 * selective terminating should be defined in each subsystem with different
 * binary representaions. e.g. BASE = 0b0001 ARM = 0b0010
 */
enum macroIds : unsigned int { DEFAULT = 0b1, BASE = 0b10 };

extern std::shared_ptr<okapi::Motor> baseFL;
extern std::shared_ptr<okapi::Motor> baseFR;

extern std::shared_ptr<okapi::OdomChassisController> base;

extern std::shared_ptr<okapi::AsyncMeshMpPpController> profileBaseController;

extern std::shared_ptr<okapi::Motor> intakeL;
extern std::shared_ptr<okapi::Motor> intakeR;
extern std::shared_ptr<okapi::MotorGroup> intake;

extern std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod>
    profileIntakeController;

extern std::shared_ptr<okapi::Motor> conveyorL;
extern std::shared_ptr<okapi::Motor> conveyorR;
extern std::shared_ptr<okapi::MotorGroup> conveyor;

extern std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod>
    profileConveyorController;

#endif // _DEFS_HPP_
