#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),
      "/ser/sout", // Output to the PROS terminal
      okapi::Logger::LogLevel::warn));
}

/**
 * Runs while the robot is in the disabled state of Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol.
 * When the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

enum macroIds : unsigned int { DEFAULT = 0b1, BASE = 0b10 };

/**
 * Configuration for the buttons of the robot with the standard syntax:
 * #define SOME_CMD_B buttonBTNID // Use suffix _B for button, _JOY for
 * joysticks
 */

/* Main Controller button configuration. */

// Intake
#define INTAKE_IN_B buttonR1
#define INTAKE_OUT_B buttonR2

// Intake
#define OUTTAKE_UP_B buttonL1
#define OUTTAKE_DWN_B buttonL2

// Misc
#define RECORD_B buttonDown

/* Main Controller joystick macros. */
#define LEFT_Y_JOY okapi::ControllerAnalog::leftY
#define LEFT_X_JOY okapi::ControllerAnalog::leftX

#define RIGHT_Y_JOY okapi::ControllerAnalog::rightY
#define RIGHT_X_JOY okapi::ControllerAnalog::rightX

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  auto base =
      okapi::ChassisControllerBuilder()
          .withMotors(16, -13, -18, 17)
          .withSensors(okapi::ADIEncoder{'A', 'B'}, okapi::ADIEncoder{'C', 'D'},
                       okapi::ADIEncoder{'E', 'F'})
          .withDimensions(okapi::AbstractMotor::gearset::green,
                          {{17.0, 17.0}, 3600})
          .withGains({0.1, 0.0001, 0.01}, // Distance controller gains
                     {0.1, 0.0001, 0.01}, // Turn controller gains
                     {0.1, 0.0001, 0.01}  // Angle controller gains
                     )
          .withOdometry({{17.0, 17.0}, 3600})
          .buildOdometry();

  okapi::MotorGroup intake({-1, 2});

  okapi::MotorGroup outtake({9, -10});

  okapi::Controller master(okapi::ControllerId::master);

  okapi::ControllerButton buttonRight(okapi::ControllerDigital::right);
  okapi::ControllerButton buttonUp(okapi::ControllerDigital::up);
  okapi::ControllerButton buttonLeft(okapi::ControllerDigital::left);
  okapi::ControllerButton buttonDown(okapi::ControllerDigital::down);
  okapi::ControllerButton buttonA(okapi::ControllerDigital::A);
  okapi::ControllerButton buttonX(okapi::ControllerDigital::X);
  okapi::ControllerButton buttonY(okapi::ControllerDigital::Y);
  okapi::ControllerButton buttonB(okapi::ControllerDigital::B);
  okapi::ControllerButton buttonL1(okapi::ControllerDigital::L1);
  okapi::ControllerButton buttonL2(okapi::ControllerDigital::L2);
  okapi::ControllerButton buttonR1(okapi::ControllerDigital::R1);
  okapi::ControllerButton buttonR2(okapi::ControllerDigital::R2);

  // Stores the currently running macro subsystems using the macro::subsystems
  // enum and bitwise opperations
  unsigned int currentlyUsedMacroSubsystems = 0b0;

  // Used for breaking the base macros if a controller joystick exceeds a
  // certain value
  const float joyMacroBreakThresh = 0.3;

  base->getModel()->resetSensors();

  base->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  while (true) {
    // Operator control code start here:

    /////////////////////////////////////////MACRO/////////////////////////////////////////////////////////
    // Macro controll and handeling:
    currentlyUsedMacroSubsystems = penvex::macro::getAllUsedSubsystems();

    if (buttonUp.changedToReleased())
      runMacroTest();
    // if (ARM_LOW_BTN.changedToReleased())
    //   runMacroLowTower();
    // if (ARM_MED_BTN.changedToReleased())
    //   runMacroMedTower();

    /////////////////////////////////////////BASE/////////////////////////////////////////////////////////
    ////----------MACRO----
    if (currentlyUsedMacroSubsystems & BASE) {
      if (fabs(master.getAnalog(LEFT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(LEFT_X_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_X_JOY)) >= joyMacroBreakThresh)
        penvex::macro::breakMacros(BASE);
    } else {
      // Drive Control:
      (std::dynamic_pointer_cast<okapi::XDriveModel>(base->getModel()))
          ->xArcade(master.getAnalog(LEFT_X_JOY), master.getAnalog(LEFT_Y_JOY),
                    master.getAnalog(RIGHT_X_JOY), 0);
    } ////----MACRO----

    /////////////////////////////////////////////INTAKE//////////////////////////////////////////////////
    intake.moveVelocity(200 *
                        (INTAKE_IN_B.isPressed() - INTAKE_OUT_B.isPressed()));

    /////////////////////////////////////////////OUTTAKE/////////////////////////////////////////////////
    outtake.moveVelocity(
        600 * (OUTTAKE_UP_B.isPressed() - OUTTAKE_DWN_B.isPressed()));

    //////////////////////////////////////////////MISC///////////////////////////////////////////////////

    // Acavate the record function
    if (RECORD_B.changedToReleased()) {
      // startRecordingVI();
    }

    // To allow for other threads to run.
    pros::Task::delay(20);
  }
}
