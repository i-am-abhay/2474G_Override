#include "vex.h"
#include "robot-config.h"
#include <cmath>

using namespace vex;

competition Competition;



// Section 1: Alliance Selection
// Switch MY_ALLIANCE between ALLIANCE_RED and ALLIANCE_BLUE before each match
#define ALLIANCE_RED   1
#define ALLIANCE_BLUE  2

const int MY_ALLIANCE = ALLIANCE_BLUE;  // Change to ALLIANCE_RED when playing on the red alliance



// Section 2: Rear Toggle System Settings (2 green-cartridge motors)
const bool   AUTO_TOGGLE_ENABLED   = true;  // Set to false to turn off automatic optical toggling

// Hue ranges (0-360 degrees) used to identify toggle colors under the optical LED
const double RED_HUE_LOW           = 330.0; // Red wraps around from 360 to 0 degrees
const double RED_HUE_HIGH          = 38.0;
const double YELLOW_HUE_LOW        = 38.1;  // Neutral yellow toggle hue range
const double YELLOW_HUE_HIGH       = 85.0;
const double BLUE_HUE_LOW          = 185.0; // Blue toggle hue range
const double BLUE_HUE_HIGH         = 255.0;

// Minimum brightness needed to detect a toggle when it is a few inches away
const double BRIGHTNESS_MIN        = 15.0;

// Motor speeds and timing values for manual and automatic toggling
const double AUTO_TOGGLE_SPEED     = 100.0; // Run at full speed during automatic color correction
const double MANUAL_TOGGLE_SPEED   = 60.0;  // Run at 60% speed when holding Button A manually
const int    TOGGLE_TIMEOUT_MS     = 2000;  // Maximum time (in ms) allowed for one auto-toggle attempt
const int    TOGGLE_COOLDOWN_MS    = 50;    // Short delay (in ms) before re-triggering during endgame battles



// Section 3: Drivetrain, Controller Curve, and Anti-Tip Settings
const int    DEADBAND              = 5;     // Ignores small joystick drift under 5%
const double DRIVE_CURVE           = 0.75;  // Blends linear and cubic input for smoother slow driving
const double TURN_CURVE            = 0.55;  // Curve weight for turning sensitivity

const double TILT_MAX              = 5.0;   // Maximum pitch angle (in degrees) before anti-tip activates
const double UNTILT_SPEED          = 60.0;  // Drive speed (%) used to balance the robot if it starts tipping

// Pneumatic claw solenoid states (swap true and false if open and close are backwards)
const bool   CLAW_OPEN             = false;
const bool   CLAW_CLOSED           = true;



// Helper Functions

// Applies a blended cubic curve to joystick values so small stick movements
// give finer control while full stick still reaches 100% power
double adaptiveCurve(double rawInput, double curveWeight) {
  if (std::abs(rawInput) < DEADBAND) {
    return 0.0;
  }
  double norm = rawInput / 100.0;
  double curved = ((1.0 - curveWeight) * norm) + (curveWeight * std::pow(norm, 3));
  return curved * 100.0;
}

// Keeps motor speed values within the valid -100% to 100% range
double clampSpeed(double speed) {
  if (speed > 100.0) return 100.0;
  if (speed < -100.0) return -100.0;
  return speed;
}

// Opens the pneumatic claw
void openClaw() {
  ClawSolenoid.set(CLAW_OPEN);
}

// Closes the pneumatic claw
void closeClaw() {
  ClawSolenoid.set(CLAW_CLOSED);
}

// Calculates left and right wheel speeds for split arcade drive
void setDrivePower(double forwardPower, double turnPower) {
  double leftSideSpeed  = clampSpeed(forwardPower - turnPower);
  double rightSideSpeed = clampSpeed(forwardPower + turnPower);

  FrontLeft.spin(forward, leftSideSpeed, percent);
  BackLeft.spin(forward, leftSideSpeed, percent);
  FrontRight.spin(forward, rightSideSpeed, percent);
  BackRight.spin(forward, rightSideSpeed, percent);
}

// Spins or stops both rear toggle motors at the same velocity
void setBothToggleMotors(double velocity) {
  if (velocity == 0.0) {
    ToggleLeft.stop(hold);
    ToggleRight.stop(hold);
  } else {
    ToggleLeft.spin(forward, velocity, percent);
    ToggleRight.spin(forward, velocity, percent);
  }
}

// Checks if the detected hue falls in the red range
bool isHueRed(double hue) {
  return (hue >= RED_HUE_LOW || hue <= RED_HUE_HIGH);
}

// Checks if the detected hue falls in the blue range
bool isHueBlue(double hue) {
  return (hue >= BLUE_HUE_LOW && hue <= BLUE_HUE_HIGH);
}

// Checks if the detected hue falls in the yellow range
bool isHueYellow(double hue) {
  return (hue >= YELLOW_HUE_LOW && hue <= YELLOW_HUE_HIGH);
}

// Returns true if a toggle is close enough for the optical sensor to read accurately
bool isToggleInRange() {
  return OpticalBack.isNearObject() || (OpticalBack.brightness() >= BRIGHTNESS_MIN);
}

// Checks if the toggle in front of the sensor is yellow or the opposing alliance color
bool backToggleNeedsFlip() {
  if (!isToggleInRange()) {
    return false;
  }

  double hue = OpticalBack.hue();
  if (MY_ALLIANCE == ALLIANCE_RED) {
    return (isHueBlue(hue) || isHueYellow(hue));
  } else {
    return (isHueRed(hue) || isHueYellow(hue));
  }
}

// Confirms whether the toggle in front of the sensor matches our alliance color
bool backToggleIsMyAlliance() {
  if (!isToggleInRange()) {
    return false;
  }

  double hue = OpticalBack.hue();
  if (MY_ALLIANCE == ALLIANCE_RED) {
    return isHueRed(hue);
  } else {
    return isHueBlue(hue);
  }
}

// Controls both toggle motors using manual Button A override or automatic optical sensing
bool handleToggleSystem() {
  static bool autoToggleRunning = false;
  static uint32_t autoToggleStart = 0;
  static uint32_t lastAutoToggleEnd = 0;

  // Manual override: holding Button A runs both toggle motors at 60% speed
  if (ControllerMain.ButtonA.pressing()) {
    autoToggleRunning = false;
    setBothToggleMotors(MANUAL_TOGGLE_SPEED);
    return true;
  }

  // Stop the motors if automatic toggling is disabled
  if (!AUTO_TOGGLE_ENABLED) {
    setBothToggleMotors(0.0);
    return false;
  }

  // Start an automatic flip if cooldown has passed and the toggle needs to be changed
  if (!autoToggleRunning) {
    if ((Brain.Timer.system() - lastAutoToggleEnd >= TOGGLE_COOLDOWN_MS) &&
        backToggleNeedsFlip()) {
      autoToggleRunning = true;
      autoToggleStart = Brain.Timer.system();
    }
  }

  // Keep spinning until our alliance color is detected, the toggle leaves range, or time runs out
  if (autoToggleRunning) {
    bool timedOut = (Brain.Timer.system() - autoToggleStart >= TOGGLE_TIMEOUT_MS);

    if (isToggleInRange() && !backToggleIsMyAlliance() && !timedOut) {
      setBothToggleMotors(AUTO_TOGGLE_SPEED);
      return true;
    } else {
      setBothToggleMotors(0.0);
      autoToggleRunning = false;
      lastAutoToggleEnd = Brain.Timer.system();
      return false;
    }
  } else {
    setBothToggleMotors(0.0);
    return false;
  }
}

// Displays optical sensor readings and toggle status on the V5 Brain screen for testing
void printSensorDebug(bool togglesActive) {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Alliance: %s   ", (MY_ALLIANCE == ALLIANCE_RED) ? "RED " : "BLUE");

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("InRange:  %s   ", isToggleInRange() ? "YES" : "NO ");

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Hue:      %.1f deg   ", OpticalBack.hue());

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Bright:   %.1f %%     ", OpticalBack.brightness());

  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Toggles:  %s   ", togglesActive ? "ON " : "OFF");
}



// Competition Functions

void pre_auton(void) {
  vexcodeInit();
  openClaw();

  // Turn on the rear optical sensor LED at full brightness
  OpticalBack.setLightPower(100, percent);
  OpticalBack.setLight(ledState::on);

  // Print alliance color, battery level, and driver reminder on the controller screen
  ControllerMain.Screen.clearScreen();
  wait(50, msec);
  ControllerMain.Screen.setCursor(1, 1);
  ControllerMain.Screen.print("Alliance: %s", (MY_ALLIANCE == ALLIANCE_RED) ? "RED" : "BLUE");
  wait(50, msec);
  ControllerMain.Screen.setCursor(2, 1);
  ControllerMain.Screen.print("Bat: %d%%", Brain.Battery.capacity());
  wait(50, msec);
  ControllerMain.Screen.setCursor(3, 1);
  ControllerMain.Screen.print("A TO TOGGLE JESTIN!");
}

void autonomous(void) {
  // Autonomous routine will be added here
}

void usercontrol(void) {
  // Assign R1 to open the claw and R2 to close it
  ControllerMain.ButtonR1.pressed(openClaw);
  ControllerMain.ButtonR2.pressed(closeClaw);

  int debugLoopCounter = 0;

  while (true) {


    // 1. Run manual and automatic rear toggle controls
    bool togglesActive = handleToggleSystem();


    // 2. Read joysticks and apply response curves (Axis 3 = forward/back, Axis 1 = turn)
    double rawForward = ControllerMain.Axis3.position();
    double rawTurn    = ControllerMain.Axis1.position();

    double yPower = adaptiveCurve(rawForward, DRIVE_CURVE);
    double xPower = adaptiveCurve(rawTurn, TURN_CURVE);


    // 3. Check inertial sensor pitch and counter-drive if the robot tilts too far
    double robotPitch = InertialSensor.pitch(degrees);
    if (robotPitch > TILT_MAX) {
      yPower = UNTILT_SPEED;
    } else if (robotPitch < -TILT_MAX) {
      yPower = -UNTILT_SPEED;
    }

    setDrivePower(yPower, xPower);


    // 4. Control the DR4B lift (L1 = up, L2 = down with bottom limit check, hold when released)
    bool raiseLift = ControllerMain.ButtonL1.pressing();
    bool lowerLift = ControllerMain.ButtonL2.pressing();

    if (raiseLift && !lowerLift) {
      LiftLeft.spin(forward, 100, percent);
      LiftRight.spin(forward, 100, percent);
    } else if (lowerLift && !raiseLift &&
               LiftLeft.position(degrees) > 0 &&
               LiftRight.position(degrees) > 0) {
      LiftLeft.spin(reverse, 100, percent);
      LiftRight.spin(reverse, 100, percent);
    } else {
      LiftLeft.stop(hold);
      LiftRight.stop(hold);
    }


    // 5. Refresh the Brain screen sensor readout every 100 ms
    debugLoopCounter++;
    if (debugLoopCounter >= 5) {
      printSensorDebug(togglesActive);
      debugLoopCounter = 0;
    }

    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
