/*
 * \file penvexlib/macros.hpp
 *
 * Contains prototypes and enums for macro functionality.
 */

#ifndef _MACROS_HPP_
#define _MACROS_HPP_

#include "api.h"

namespace penvex {

class Macro {
public:
  Macro(unsigned int iusedSubsystems, void (*imacroFunction)(void *),
        bool irestartOnBreak, void *ifuncParams = NULL,
        std::uint32_t iprio = TASK_PRIORITY_DEFAULT,
        std::uint16_t istack_depth = TASK_STACK_DEPTH_DEFAULT,
        const char *iname = "");

  /**
   * Initalizes the macro runner - call once in init
   */
  static void initRunner(unsigned int inumberOfSubsystems);

  /**
   * Stops the currently running macros whose subsystems conflict with the
  subsystems given.
   *
   * \Param subsystemsToBreak The ids of the subsystems that if a currently
   * running macro matches will cause the running macro to terminate.
   */
  static void breakMacros(unsigned int subsystemsToBreak);

  /**
   * \Return a pointer to the a filled running macro array.
   */
  static Macro **getRunningMacroData(Macro *fillArr[]);

  /**
   * Get all used subsystems as an ID musing the macro::subsystems enum.
   *
   * \Return bitwise or all cuttent used subsystems.
   */
  static unsigned int getAllUsedSubsystems();

  /**
   * Generates the macro task - do this in init to save time - eepecaly for no
   * repeat macros.
   */
  void init();

  /**
   * Run the macro.
   *
   * \Param macroFuncParams pointer to the pramas given to the macro, default to
   * NULL
   */
  void run(void *macroFuncParams = NULL);

  /**
   * This should only be called at the end of a restart macro to remove it form
   * the list of running macros.
   */
  void remove();

  /**
   * Gets the subsystems for the macro that this is called upon.
   */
  unsigned int getUsedSubsystems();

private:
  /**
   * The count of subsystems the macro involves - to avoid competition and allow
   * for selective terminating should be defined in each subsystem with
   * different binary representaions. e.g. BASE = 0b0001 ARM = 0b0010
   */
  static unsigned int numberOfSubsystems;

  /**
   * The array containging all currently running macros - note: not all existant
   * macros.
   */
  static Macro **runningMacros;

  /**
   * The mutex to invoke when accessing the static members.
   */
  static pros::Mutex runningMacroMutex;

  // The data stored in a macro
  unsigned int usedSubsystems;
  void (*macroFunction)(void *);
  bool restartOnBreak;
  void *funcParams;
  std::uint32_t prio;
  std::uint16_t stack_depth;
  const char *name;
  pros::Task *macroTask;
};

} // namespace penvex

#endif
