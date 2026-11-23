#include "vex.h"
#include "robot-config.h"
#include <cmath>

using namespace vex;

competition Competition;



// 1. ALLIANCE SELECTION (Change this line between ALLIANCE_RED / ALLIANCE_BLUE)
#define ALLIANCE_RED   1
#define ALLIANCE_BLUE  2

const int MY_ALLIANCE = ALLIANCE_BLUE;  // <-- CHANGE TO ALLIANCE_RED WHEN ON RED



// 2. TOGGLE SETTINGS (2 Rear Green-Cartridge Toggle Motors)
const bool   AUTO_TOGGLE_ENABLED   = true;  // Set false to disable optical auto-toggle

// Optical Hue Ranges (0 - 360 degrees; widened to prevent red/orange LED dead-zones)
const double RED_HUE_LOW           = 330.0; // Red wraps around 360 -> 0
const double RED_HUE_HIGH          = 38.0;
const double YELLOW_HUE_LOW        = 38.1;  // Neutral yellow toggle range
const double YELLOW_HUE_HIGH       = 85.0;
const double BLUE_HUE_LOW          = 185.0; // Blue toggle range
const double BLUE_HUE_HIGH         = 255.0;

// Proximity & Brightness Thresholds
const double BRIGHTNESS_MIN        = 15.0;  // Triggers if object is within ~3-4 in. even if isNearObject() is false

// Toggle Motor Velocities & Timers
const double AUTO_TOGGLE_SPEED     = 100.0; // Full 100% velocity for automated optical toggling
const double MANUAL_TOGGLE_SPEED   = 60.0;  // 60% velocity when holding circular ButtonA
const int    TOGGLE_TIMEOUT_MS     = 2000;  // Safety timeout (ms) per flip attempt
const int    TOGGLE_COOLDOWN_MS    = 50;    // Near-instant (50ms) recovery for endgame dog fights



// 3. ADAPTIVE SPLIT ARCADE & ANTI-TIP SETTINGS
const int    DEADBAND              = 5;     // Ignore stick drift below 5%
const double DRIVE_CURVE           = 0.75;  // 0.0 = linear, 1.0 = full cubic (0.75 = slow fine creep)
const double TURN_CURVE            = 0.55;  // Responsive turn curve

const double TILT_MAX              = 5.0;   // Pitch threshold (degrees) for anti-tip
const double UNTILT_SPEED          = 60.0;  // Counter-drive speed (%)

// Pneumatic Claw Polarity (swap true/false if R1/R2 are inverted)
const bool   CLAW_OPEN             = false;
const bool   CLAW_CLOSED           = true;



// HELPER FUNCTIONS

/**
 * Adaptive blended cubic curve: slight stick movement drives slowly,
 * full stick reaches 100% power.
 */
double adaptiveCurve(double rawInput, double curveWeight) {
  if (std::abs(rawInput) < DEADBAND) {
    return 0.0;
  }
  double norm = rawInput / 100.0;
  double curved = ((1.0 - curveWeight) * norm) + (curveWeight * std::pow(norm, 3));
  return curved * 100.0;
}

double clampSpeed(double speed) {
  if (speed > 100.0) return 100.0;
  if (speed < -100.0) return -100.0;
  return speed;
}

void openClaw() {
  ClawSolenoid.set(CLAW_OPEN);
}

void closeClaw() {
  ClawSolenoid.set(CLAW_CLOSED);
}

/**
 * Drives the chassis using front-facing (claw-view) Left/Right naming.
 */
void setDrivePower(double forwardPower, double turnPower) {
  double leftSideSpeed  = clampSpeed(forwardPower - turnPower);
  double rightSideSpeed = clampSpeed(forwardPower + turnPower);

  FrontLeft.spin(forward, leftSideSpeed, percent);
  BackLeft.spin(forward, leftSideSpeed, percent);
  FrontRight.spin(forward, rightSideSpeed, percent);
  BackRight.spin(forward, rightSideSpeed, percent);
}

/**
 * Turns both rear toggle motors on or off together at the same velocity.
 */
void setBothToggleMotors(double velocity) {
  if (velocity == 0.0) {
    ToggleLeft.stop(hold);
    ToggleRight.stop(hold);
  } else {
    ToggleLeft.spin(forward, velocity, percent);
    ToggleRight.spin(forward, velocity, percent);
  }
}

// Color classification helpers
bool isHueRed(double hue) {
  return (hue >= RED_HUE_LOW || hue <= RED_HUE_HIGH);
}

bool isHueBlue(double hue) {
  return (hue >= BLUE_HUE_LOW && hue <= BLUE_HUE_HIGH);
}

bool isHueYellow(double hue) {
  return (hue >= YELLOW_HUE_LOW && hue <= YELLOW_HUE_HIGH);
}

/**
 * Checks whether a toggle is close enough to read reliably.
 */
bool isToggleInRange() {
  return OpticalBack.isNearObject() || (OpticalBack.brightness() >= BRIGHTNESS_MIN);
}

/**
 * Returns true when OpticalBack detects a toggle in range AND its color is
 * either Yellow OR the opposite alliance color.
 */
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

/**
 * Returns true once OpticalBack confirms the toggle in range is MY_ALLIANCE.
 */
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

/**
 * Runs both toggle motors at 60% when circular ButtonA is held, or at 100%
 * automatically whenever OpticalBack detects the toggle is yellow or the
 * opponent's color—continuously reclaiming MY_ALLIANCE during dog fights.
 */
bool handleToggleSystem() {
  static bool autoToggleRunning = false;
  static uint32_t autoToggleStart = 0;
  static uint32_t lastAutoToggleEnd = 0;

  // 1. MANUAL CIRCULAR BUTTON OVERRIDE (ButtonA turns on both toggle motors at 60%)
  if (ControllerMain.ButtonA.pressing()) {
    autoToggleRunning = false;
    setBothToggleMotors(MANUAL_TOGGLE_SPEED);
    return true;
  }

  // 2. BACK OPTICAL AUTO-TOGGLE (Runs both toggle motors at 100% until MY_ALLIANCE)
  if (!AUTO_TOGGLE_ENABLED) {
    setBothToggleMotors(0.0);
    return false;
  }

  if (!autoToggleRunning) {
    if ((Brain.Timer.system() - lastAutoToggleEnd >= TOGGLE_COOLDOWN_MS) &&
        backToggleNeedsFlip()) {
      autoToggleRunning = true;
      autoToggleStart = Brain.Timer.system();
    }
  }

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

/**
 * Prints live OpticalBack values on the V5 Brain screen for easy tuning.
 */
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



// COMPETITION CALLBACKS

void pre_auton(void) {
  vexcodeInit();
  openClaw();

  // Ensure Back Optical LED is turned on at 100%
  OpticalBack.setLightPower(100, percent);
  OpticalBack.setLight(ledState::on);

  // Display active alliance, battery, and toggle reminder on controller screen
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
  // Autonomous routine goes here
}

void usercontrol(void) {
  // Pneumatic Claw Controls (R1 = Open, R2 = Close)
  ControllerMain.ButtonR1.pressed(openClaw);
  ControllerMain.ButtonR2.pressed(closeClaw);

  int debugLoopCounter = 0;

  while (true) {


    // 1. TOGGLE MOTORS (Circular Button A at 60% + Back Optical Auto at 100%)
    bool togglesActive = handleToggleSystem();


    // 2. ADAPTIVE SPLIT ARCADE DRIVETRAIN (Axis 3 = Fwd/Back, Axis 1 = Turn)
    double rawForward = ControllerMain.Axis3.position();
    double rawTurn    = ControllerMain.Axis1.position();

    double yPower = adaptiveCurve(rawForward, DRIVE_CURVE);
    double xPower = adaptiveCurve(rawTurn, TURN_CURVE);


    // 3. INERTIAL ANTI-TIP OVERRIDE
    double robotPitch = InertialSensor.pitch(degrees);
    if (robotPitch > TILT_MAX) {
      yPower = UNTILT_SPEED;
    } else if (robotPitch < -TILT_MAX) {
      yPower = -UNTILT_SPEED;
    }

    setDrivePower(yPower, xPower);


    // 4. DR4B LIFT MANUAL CONTROL (L1 = Raise, L2 = Lower, Hold when released)
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


    // 5. LIVE BRAIN SCREEN DEBUG (Updates every 100ms)
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
