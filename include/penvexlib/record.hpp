/*
 * \file penvexlib/record.hpp
 *
 * Contrins functions for recording the movement of the robot to repay in a
 * motion profile controller.
 */

#ifndef _RECORD_HPP_
#define _RECORD_HPP_

#include "penvexlib/macros.hpp"

namespace penvex::record {

extern const int RECORD_TIME;    // Period of time of one recording
extern const int MSEC_PER_FRAME; // the time between frimes in msec
extern const int BYTES_RECORDED_PER_FRAME;

extern penvex::macro::macroData recordMacro_data;
extern void runRecordMacro();
extern void initRecordMacro();

} // namespace penvex::record

#endif
