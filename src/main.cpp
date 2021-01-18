#include "main.h"

// Defs:
const unsigned int penvex::macro::numberOfSubsystems = 2;

// BASE:

std::shared_ptr<okapi::Motor> baseFL;
std::shared_ptr<okapi::Motor> baseFR;

std::shared_ptr<okapi::OdomChassisController> base;

std::shared_ptr<okapi::AsyncMeshMpPpController> profileBaseController;

// INTAKE:

std::shared_ptr<okapi::Motor> intakeL;
std::shared_ptr<okapi::Motor> intakeR;
std::shared_ptr<okapi::MotorGroup> intake;

std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod>
    profileIntakeController;

// CONVEYOR:

std::shared_ptr<okapi::Motor> conveyorL;
std::shared_ptr<okapi::Motor> conveyorR;
std::shared_ptr<okapi::MotorGroup> conveyor;

std::shared_ptr<okapi::AsyncLinearMotionProfileControllerMod>
    profileConveyorController;

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
      okapi::Logger::LogLevel::info));

  // BASE init:

  baseFL = std::make_shared<okapi::Motor>(-19);
  baseFR = std::make_shared<okapi::Motor>(17);

  using namespace okapi::literals;
  okapi::ChassisScales baseScales(
      {2.75_in, 14.3125_in, 5.1875_in, 4.58333333_in}, 600);

  okapi::ChassisScales baseScalesHack(
      {8.25_in, 14.3125_in, 5.1875_in, 4.58333333_in}, 600);

  // NOTE: Okapi didnt properly implement GearsetRatioPair into chassis
  // ChassisControllerBuilder - I am just telling it the wheels are three times
  // as big
  base = okapi::ChassisControllerBuilder()
             .withMotors({-20, -19}, {18, 17})
             .withSensors(okapi::IntegratedEncoder{19, true},
                          okapi::IntegratedEncoder{17, false},
                          okapi::ADIEncoder{'G', 'H', true})
             .withDimensions(okapi::AbstractMotor::gearset::red, baseScalesHack)
             .withGains({0.001, 0.0, 0.0}, // Distance controller gains
                        {0.0, 0.0, 0.0},   // Turn controller gains
                        {0.0, 0.0, 0.0}    // Angle controller gains
                        )
             .withOdometry(baseScales)
             .buildOdometry();

  okapi::PurePursuitConstants constants{0.4, 8.0_in, 5_in, 0.7_in};

  // NOTE: This might cause init to run out of stack space
  profileBaseController =
      okapi::AsyncMotionProfileControllerBuilderMod()
          .withOutput(base)
          .withLimits({0.5, 1.0, 8.0})
          .withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory())
          .withOdometry(base->getOdometry(), &constants)
          .buildMeshMpPpController();
  pros::Task::delay(10);

  // INTAKE init:

  intakeL = std::make_shared<okapi::Motor>(10);
  intakeR = std::make_shared<okapi::Motor>(-15);

  intake = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<std::shared_ptr<okapi::AbstractMotor>>{intakeL,
                                                                   intakeR});

  profileIntakeController =
      okapi::AsyncMotionProfileControllerBuilderMod()
          .withOutput(intake, 3.5_in,
                      okapi::AbstractMotor::GearsetRatioPair(
                          okapi::AbstractMotor::gearset::green))
          .withLimits({0.5, 1.0, 8.0})
          .buildLinearMotionProfileControllerMod();
  pros::Task::delay(10);

  // printf("test\n");
  // pros::Task::delay(10);

  // CONVEYOR init:

  conveyorL = std::make_shared<okapi::Motor>(-9);
  conveyorR = std::make_shared<okapi::Motor>(14);

  conveyor = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<std::shared_ptr<okapi::AbstractMotor>>{conveyorL,
                                                                   conveyorR});

  profileConveyorController =
      okapi::AsyncMotionProfileControllerBuilderMod()
          .withOutput(conveyor, 3.0_in,
                      okapi::AbstractMotor::GearsetRatioPair(
                          okapi::AbstractMotor::gearset::blue))
          .withLimits({0.5, 1.0, 8.0})
          .buildLinearMotionProfileControllerMod();
  pros::Task::delay(10);

  scripts::initMacroTest();
  scripts::initMacroTest2();
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
#define CONVEYOR_UP_B buttonL1
#define CONVEYOR_DWN_B buttonL2

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

  // okapi::ChassisScales baseScales(
  //     {(5570 / 2.05105), 8.63636363637, 0.1317625, (3200 / 1.9558)}, 1800);

  okapi::IntegratedEncoder testL{19, true};
  okapi::IntegratedEncoder testR{17, false};
  okapi::ADIEncoder testM{'G', 'H', true};

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
  const float joyMacroBreakThresh = 0.1;

  base->getModel()->resetSensors();

  base->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  base->startOdomThread();

  std::string testStr;

  while (true) {
    // Operator control code start here:

    /////////////////////////////////////////MACRO/////////////////////////////////////////////////////////
    // Macro controll and handeling:
    currentlyUsedMacroSubsystems = penvex::macro::getAllUsedSubsystems();

    if (buttonUp.changedToReleased())
      scripts::runMacroTest();

    // if (buttonDown.changedToReleased()) {
    //   // penvex::macro::breakMacros(0b01);
    //   testStr = base->getOdometry()->getState().str();
    //   printf("%s\n", testStr.c_str());
    //   // printf("%d\n", currentlyUsedMacroSubsystems);
    // }

    if (buttonLeft.changedToReleased())
      scripts::runMacroTest2();

    if (buttonRight.changedToReleased())
      penvex::macro::breakMacros(0b10);

    /////////////////////////////////////////BASE/////////////////////////////////////////////////////////
    ////----------MACRO----
    if (currentlyUsedMacroSubsystems & BASE) {
      if (fabs(master.getAnalog(LEFT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(LEFT_X_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_X_JOY)) >= joyMacroBreakThresh) {
        penvex::macro::breakMacros(BASE);
        base->stop();
        profileBaseController->flipDisable(true);
      }
    } else {
      (std::dynamic_pointer_cast<okapi::SkidSteerModel>(base->getModel()))
          ->tank(master.getAnalog(LEFT_Y_JOY), master.getAnalog(RIGHT_Y_JOY));
    } ////----MACRO----

    /////////////////////////////////////////////INTAKE//////////////////////////////////////////////////
    intake->moveVoltage(12000 *
                        (INTAKE_IN_B.isPressed() - INTAKE_OUT_B.isPressed()));

    /////////////////////////////////////////////CONVEYOR/////////////////////////////////////////////////
    conveyor->moveVoltage(
        12000 * (CONVEYOR_UP_B.isPressed() - CONVEYOR_DWN_B.isPressed()));

    //////////////////////////////////////////////MISC///////////////////////////////////////////////////

    // Acavate the record function
    if (RECORD_B.changedToReleased()) {
      penvex::record::runRecordMacro();
      // startRecordingVI();
    }

    // To allow for other threads to run.
    pros::Task::delay(20);
  }
}
