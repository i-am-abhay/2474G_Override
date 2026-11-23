#include "vex.h"
#include "robot-config.h"

using namespace vex;

// Global V5 Brain and primary controller objects
brain Brain;
controller ControllerMain = controller(primary);



// Drivetrain Motors (based on facing the front claw)
// Positive power spins all four wheels forward toward the front claw
motor FrontLeft  = motor(FRONT_LEFT_PORT,  ratio6_1, false);
motor BackLeft   = motor(BACK_LEFT_PORT,   ratio6_1, false);
motor FrontRight = motor(FRONT_RIGHT_PORT, ratio6_1, true);
motor BackRight  = motor(BACK_RIGHT_PORT,  ratio6_1, true);



// DR4B Lift Motors (based on facing the front claw)
motor LiftLeft   = motor(LIFT_LEFT_PORT,   ratio18_1, false);
motor LiftRight  = motor(LIFT_RIGHT_PORT,  ratio18_1, true);



// Rear Toggle Motors (200 RPM green cartridges)
// Polarity is set so positive speed spins both toggle motors in the proper direction
motor ToggleLeft  = motor(TOGGLE_LEFT_PORT,  ratio18_1, true);
motor ToggleRight = motor(TOGGLE_RIGHT_PORT, ratio18_1, false);



// Sensors and Pneumatic Claw Solenoid
inertial InertialSensor = inertial(INERTIAL_PORT);
optical OpticalBack = optical(OPTICAL_BACK_PORT);
digital_out ClawSolenoid = digital_out(CLAW_SOLENOID_PORT);

void vexcodeInit(void) {
  // Set the DR4B lift motors to hold their position when L1 or L2 are released
  LiftLeft.setBrake(hold);
  LiftRight.setBrake(hold);
  LiftLeft.setVelocity(100, percent);
  LiftRight.setVelocity(100, percent);
  LiftLeft.setPosition(0, degrees);
  LiftRight.setPosition(0, degrees);

  // Keep the rear toggle motors in hold mode when they are not spinning
  ToggleLeft.setBrake(hold);
  ToggleRight.setBrake(hold);

  // Turn on the rear optical sensor LED at 100% brightness for consistent color readings
  OpticalBack.setLightPower(100, percent);
  OpticalBack.setLight(ledState::on);

  // Calibrate the inertial sensor at startup (keep the robot completely still)
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(20, msec);
  }
}
