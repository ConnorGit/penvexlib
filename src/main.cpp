#include "main.h"

using namespace okapi::literals;

// Defs:
const unsigned int numberOfSubsystems = 5;

// BASE:

std::shared_ptr<okapi::Motor> baseFL;
std::shared_ptr<okapi::Motor> baseFR;

std::shared_ptr<okapi::OdomChassisController> base;

std::shared_ptr<okapi::AsyncMeshMpPpController> profileBaseController;

std::shared_ptr<okapi::IMU> imuZ;

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

void resetData() {
  base->getModel()->resetSensors();
  intake->tarePosition();
  conveyor->tarePosition();
  base->setState(okapi::OdomState{0.2032_m, 0.8636_m, 0.0_deg});
}

void intakeMoveVoltage(int voltage) {
  int dif = (int)((2.0) * (intakeR->getPosition() - intakeL->getPosition()));
  // I'm bad at code - the voltage bit corrects for driving backwards with XOR
  if ((dif >= 0) == (voltage >= 0)) {
    intakeR->moveVoltage(voltage - dif);
    intakeL->moveVoltage(voltage);
  } else {
    intakeR->moveVoltage(voltage);
    intakeL->moveVoltage(voltage + dif);
  }
}

void intakeMoveVelocity(int vel) {
  int dif = (int)((0.2) * (intakeR->getPosition() - intakeL->getPosition()));
  // I'm bad at code - the vel bit corrects for driving backwards with XOR
  if ((dif >= 0) == (vel >= 0)) {
    intakeR->moveVelocity(vel - dif);
    intakeL->moveVelocity(vel);
  } else {
    intakeR->moveVelocity(vel);
    intakeL->moveVelocity(vel + dif);
  }
}

double crapOdomStartOffTheta = 0.0;
void towerResetOdom(char towerID) {
  // Everything in meters
  int indexNum = (int)towerID - 65;
  double towerPosArr[9][2] = {
      {3.49504, 3.49504}, {1.8288, 3.51155}, {0.16256, 3.49504},
      {3.51155, 1.8288},  {1.8288, 1.8288},  {0.14605, 1.8288},
      {3.49504, 0.16256}, {1.8288, 0.14605}, {0.16256, 0.16256}};
  double botStopDis = 0.22;

  double towerX = towerPosArr[indexNum][0];
  double towerY = towerPosArr[indexNum][1];

  double currentTheta = imuZ->getRemapped(-PI, PI) + crapOdomStartOffTheta;

  double newX = towerX - (botStopDis * cos(currentTheta));
  double newY = towerY + (botStopDis * sin(currentTheta));

  base->setState(okapi::OdomState{newX * okapi::meter, newY * okapi::meter,
                                  (-currentTheta) * okapi::radian});
}

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
      okapi::Logger::LogLevel::debug));

  pros::lcd::initialize();

  // BASE init:

  baseFL = std::make_shared<okapi::Motor>(-19);
  baseFR = std::make_shared<okapi::Motor>(17);

  okapi::ChassisScales baseScales(
      {2.75_in, 14.3125_in, 5.1875_in, 4.58333333_in}, 600);

  okapi::ChassisScales baseScalesHack(
      {8.25_in, 14.3125_in, 5.1875_in, 4.58333333_in}, 600);

  // NOTE: Okapi didnt properly implement GearsetRatioPair into chassis
  // ChassisControllerBuilder - I am just telling it the wheels are three times
  // as big trn{0.0013, 0.0003, 0.00002} ang{0.00135, 0.0005, 0.00002}
  base = okapi::ChassisControllerBuilder()
             .withMotors({-20, -19}, {18, 17})
             .withSensors(okapi::IntegratedEncoder{19, true},
                          okapi::IntegratedEncoder{17, false},
                          okapi::ADIEncoder{'G', 'H', true})
             .withDimensions(okapi::AbstractMotor::gearset::red, baseScales)
             .withGains({0.00085, 0.0002, 0.00001}, // Distance controller gains
                        {0.0013, 0.0003, 0.00002},  // Turn controller gains
                        {0.00135, 0.0005, 0.00002}  // Angle controller gains
                        )
             .withClosedLoopControllerTimeUtil(2.0, 1.0, 410_ms)
             .withOdometry(baseScales)
             .buildOdometry();

  okapi::PurePursuitConstants constants{1.00, 13.0_in, 5_in, 2.2_in};

  // NOTE: This might cause init to run out of stack space
  profileBaseController =
      okapi::AsyncMotionProfileControllerBuilderMod()
          .withOutput(base)
          .withLimits({0.5, 1.0, 8.0})
          .withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory())
          .withOdometry(base->getOdometry(), &constants)
          .buildMeshMpPpController();
  pros::Task::delay(10);

  imuZ = std::make_shared<okapi::IMU>(5);

  imuZ->calibrate();

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

  penvex::Macro::initRunner(numberOfSubsystems);

  scripts::initMacroTest();
  scripts::initMacroTest2();

  penvex::record::initRecordMacro();
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
void autonomous() { penvex::master::runMasterFile("/data/final", "final"); }

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
  base->stop();
  profileBaseController->flipDisable(true);
  profileIntakeController->flipDisable(true);
  profileConveyorController->flipDisable(true);

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

  // base->getModel()->resetSensors();
  resetData();

  base->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  base->startOdomThread();

  std::string testStr;

  while (true) {
    // Operator control code start here:

    /////////////////////////////////////////MACRO/////////////////////////////////////////////////////////
    // Macro controll and handeling:
    currentlyUsedMacroSubsystems = penvex::Macro::getAllUsedSubsystems();

    if (buttonUp.changedToReleased()) {
      // okapi::OdomState cur = base->getState();
      // printf("%f  %f  %f    %f\n", cur.x.convert(okapi::meter),
      //        cur.y.convert(okapi::meter), cur.theta.convert(okapi::radian),
      //        (imuZ->getRemapped(-PI, PI) + crapOdomStartOffTheta));
      // pros::Task::delay(100);
      // scripts::macroTest->run();
    }

    if (buttonLeft.changedToReleased()) {
      scripts::macroTest2->run();
      // okapi::OdomState cur = base->getState();
      // double currentTheta = imuZ->getRemapped(-PI, PI) +
      // crapOdomStartOffTheta;
      //
      // printf("%f  %f\n",
      //        cur.x.convert(okapi::meter) + 0.2032 * cos(currentTheta),
      //        cur.y.convert(okapi::meter) - 0.2032 * sin(currentTheta));
      // pros::Task::delay(100);
    }

    // scripts::macroTest2->run();

    if (buttonRight.changedToReleased()) {
      // towerResetOdom('H');
      penvex::Macro::breakMacros(BASE | INTAKE | CONVEYOR);
      profileBaseController->flipDisable(true);
      base->stop();
    }

    /////////////////////////////////////////BASE/////////////////////////////////////////////////////////
    ////----------MACRO----
    if (currentlyUsedMacroSubsystems & BASE) {
      if (fabs(master.getAnalog(LEFT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_Y_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(LEFT_X_JOY)) >= joyMacroBreakThresh ||
          fabs(master.getAnalog(RIGHT_X_JOY)) >= joyMacroBreakThresh) {
        penvex::Macro::breakMacros(BASE);
        profileBaseController->flipDisable(true);
        base->stop();
      }
    } else {
      (std::dynamic_pointer_cast<okapi::SkidSteerModel>(base->getModel()))
          ->tank(master.getAnalog(LEFT_Y_JOY), master.getAnalog(RIGHT_Y_JOY));
    } ////----MACRO----

    /////////////////////////////////////////////INTAKE//////////////////////////////////////////////////
    // intakeMoveVoltage(12000 *
    //                   (INTAKE_IN_B.isPressed() - INTAKE_OUT_B.isPressed()));

    if (currentlyUsedMacroSubsystems & INTAKE) {
      if (INTAKE_IN_B.isPressed() || INTAKE_OUT_B.isPressed()) {
        penvex::Macro::breakMacros(INTAKE);
        profileIntakeController->flipDisable(true);
        intake->moveVoltage(0);
      }
    } else {

      intakeMoveVelocity(200 *
                         (INTAKE_IN_B.isPressed() - INTAKE_OUT_B.isPressed()));
    }

    /////////////////////////////////////////////CONVEYOR/////////////////////////////////////////////////

    if (currentlyUsedMacroSubsystems & CONVEYOR) {
      if (CONVEYOR_UP_B.isPressed() || CONVEYOR_DWN_B.isPressed()) {
        penvex::Macro::breakMacros(CONVEYOR);
        profileConveyorController->flipDisable(true);
        conveyor->moveVoltage(0);
      }
    } else {
      // if recording
      if (currentlyUsedMacroSubsystems & 0b1000) {

        conveyor->moveVelocity(
            600 * (CONVEYOR_UP_B.isPressed() - CONVEYOR_DWN_B.isPressed()));

      } else {

        conveyor->moveVoltage(
            12000 * (CONVEYOR_UP_B.isPressed() - CONVEYOR_DWN_B.isPressed()));
      }
    }

    // Acavate the record function
    if (RECORD_B.changedToReleased()) {
      penvex::record::recordMacro->run();
      // startRecordingVI();
    }

    // To allow for other threads to run.
    pros::Task::delay(20);

    // /* Just test code I want to remember */
    // if (fabs(y) > fabs(x))
    //   smallerLenOnSqr = x / y;
    // else
    //   smallerLenOnSqr = y / x;
    // changeJoyToSqrScalar = 0.99303 + (0.5 * smallerLenOnSqr *
    // smallerLenOnSqr);
  }
}
