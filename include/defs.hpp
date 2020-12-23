
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

extern std::shared_ptr<okapi::OdomChassisController> base;

extern std::shared_ptr<okapi::AsyncMeshMpPpController> profileBaseController;

#endif // _DEFS_HPP_
