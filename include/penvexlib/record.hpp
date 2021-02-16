/*
 * \file penvexlib/record.hpp
 *
 * Contrins functions for recording the movement of the robot to repay in a
 * motion profile controller.
 */

#ifndef _RECORD_HPP_
#define _RECORD_HPP_

#include "penvexlib/macros.hpp"
#include "penvexlib/masterRunner.hpp"
#include "penvexlib/mods/asyncLinearMotionProfileControllerMod.hpp"
#include "penvexlib/mods/asyncMeshMpPpController.hpp"

namespace penvex::record {

extern const int RECORD_TIME;    // Period of time of one recording
extern const int MSEC_PER_FRAME; // the time between frimes in msec
extern const int BYTES_RECORDED_PER_FRAME;

extern penvex::Macro *recordMacro;
extern void initRecordMacro();

/**
 * Returns unused file name for recording based on if the master exists.
 */
extern std::string getNewRecID(const std::string &idirectory);

/**
 * Creates a master file used to run a recauton // BAD
 */
extern void createBasicMasterFile(const std::string &idirectory,
                                  const std::string &ipathId);

extern void readMasterFile(penvex::master::masterFunction **&masterFunctionList,
                           std::string &fillDir, int &fillLen,
                           const std::string &idirectory,
                           const std::string &ipathId);
/**
 * Stores a csv full of doubles beginnig with one line containing the number of
 * segments
 * @param length                The number of segments in the arrays to store
 * @param numberOfValuesPerLine The number of arrays of values to store
 * @param arrays                An array of arrays of the doubles
 * @param idirectory            The directory name
 * @param ifileName             The file name, don't add the extension
 */

extern void storeDoubles(const int length, const int numberOfValuesPerLine,
                         const double *arrays[], const std::string &idirectory,
                         const std::string &ifileName);

/**
 * Loads into the controller a trajectory from the csv in the format: x, y,
 * velL, velR Intended to be used with store doubles.
 */

extern void
loadPath(std::shared_ptr<okapi::AsyncMeshMpPpController> &controller,
         const std::string &idirectory, const std::string &ifileName,
         const std::string &ipathId);

/**
 * Loads into the controller a trajectory from the csv in the format: vel
 * Intended to be used with store doubles.
 */

extern void loadPath(
    std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod> &controller,
    const std::string &idirectory, const std::string &ifileName,
    const std::string &ipathId);

} // namespace penvex::record

#endif
