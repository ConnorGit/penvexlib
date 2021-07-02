/**
 * \file penvexlib/auton_selector.cpp
 *
 * Selects between autonomus rutienes before match and runs the auton init
 * function.
 */

#include "main.h"

using namespace pros::lcd;
using namespace penvex;

namespace LCDSelector {

pros::Task *_LCDSelectorTask;

void _cls() {
  for (int i = 0; i < 8; i++)
    print(i, "");
}

int _lcdVectorSelector(const std::vector<std::string> choices,
                       std::string listName) {
  const int vectorSize = choices.size();
  int selectedLine = 0, offset = 0;
  std::string out, name;
  while (true) {
    while (read_buttons() != LCD_BTN_RIGHT) {

      if (read_buttons() == LCD_BTN_LEFT) {
        --selectedLine;
        selectedLine =
            selectedLine < 0
                ? (std::clamp(vectorSize - offset + (int)(vectorSize > 8), 0,
                              8) -
                   1)
                : selectedLine;
        while (read_buttons() == LCD_BTN_LEFT)
          pros::Task::delay(100);
      }
      if (read_buttons() == LCD_BTN_CENTER) {
        ++selectedLine %=
            std::clamp(vectorSize - offset + (int)(vectorSize > 8), 0, 8);
        while (read_buttons() == LCD_BTN_CENTER)
          pros::Task::delay(100);
      }

      // Print the list
      _cls();
      for (int i = 0; i < 8; i++) {
        if ((i + offset) < (vectorSize + (int)(vectorSize > 8))) {
          if ((i == 7 || i + offset == vectorSize) && vectorSize > 8)
            name = "NEXT PAGE";
          else
            name = choices[i + offset];
          if (i == selectedLine)
            out = ">> " + name + " <<\n";
          else
            out = "    " + name + "\n";
          if (i == 0) {
            for (int j = 0; j < 75 - out.size() - listName.size(); j++)
              out += " ";
            out += listName;
          }
          if (out.size() < 75)
            print(i, out.c_str());
          else
            print(i, "ERROR: String too big!");
        } else
          break;
      }

      pros::Task::delay(100);
    }
    // Go to next offset of exit loop
    if ((selectedLine == 7 || selectedLine + offset == vectorSize) &&
        vectorSize > 8) {
      offset += 7;
      offset = (offset >= vectorSize) ? 0 : offset;
      selectedLine = 0;
      while (read_buttons() == LCD_BTN_RIGHT)
        pros::Task::delay(100);
    } else
      break;
  }
  int choice = offset + selectedLine;
  _cls();
  print(0, choices[choice].c_str());
  pros::Task::delay(600);
  _cls();
  return choice;
}

//////////////////////////////////////////////////////////////////////////////////

void _LCDRunnerTaskFunc(void *param) {
  if (!pros::lcd::is_initialized()) {
    pros::lcd::initialize();
    printf("LCD wasn't initalised until LCDRunner was called");
  }
  // Select the auton side

  while (true) {

    autonSide = static_cast<fieldSides>(
        _lcdVectorSelector(std::vector<std::string>{"BLUE", "RED"}, "[side]"));

    // Select the auton function
    std::vector<std::string> autonNames;
    for (int i = 0; i < NUMBER_OF_AUTONS; i++)
      autonNames.push_back(Autons[i].name);
    int tempAutoNumber = _lcdVectorSelector(autonNames, "[auton]");
    autonFunction = Autons[tempAutoNumber].autoFuncPtr;
    autonInitFunction = Autons[tempAutoNumber].autoInitPtr;
    autonFreeDatFunction = Autons[tempAutoNumber].autoFreeDatPtr;

    print(0, "Init auto mem...");

    // This is a bit of a hack
    (*Autons[tempAutoNumber]
          .autoInitPtr)(); // Generates the paths and inits for the chosen auton

    _cls();
    print(0, "Init Fin.");
    pros::Task::delay(800);
    _cls();
    print(0, "Reset auto: RIGHT");
    while (read_buttons() != LCD_BTN_RIGHT ||
           (!pros::competition::is_disabled() &&
            pros::competition::is_connected()))
      pros::Task::delay(100);
    while (read_buttons() == LCD_BTN_RIGHT)
      pros::Task::delay(100);

    if (pros::competition::is_disabled())
      (*Autons[tempAutoNumber]
            .autoFreeDatPtr)(); // Removes the data of the prevoius auto
  }
}

/**
 * The macro object which contains both the macro function to
 * run in its own task and an identifactaion for the subsystems running in it to
 * allow the macro to break if a subsystem is interupted.
 *
 * If restart is set to true then all dynamicaly allocated memory in the macro
 * must be cleaned.
 */
// NOTE: The auton selector runs the auton Init function
Macro *autonSelectorMacro;

/**
 * This function should be called in init this macro in initalise at the start
 * of the program
 *
 * IMPORTANT - The auton selector runs the auton Init function
 */
void init() {
  autonSelectorMacro =
      new Macro(LCD, _LCDRunnerTaskFunc, false, NULL, TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT * 2,
                "AutonSelector"); // dont record at very high res for more than
                                  // like 50 sec
  autonSelectorMacro->init();
}

} // namespace LCDSelector
