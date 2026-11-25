#include "vex.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
#include <cstring>

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
const double BRIGHTNESS_MIN        = 15.0;  // Set to 15.0 so ambient lighting does not trigger false positives

// Motor speeds and timing values for manual and automatic toggling
const double AUTO_TOGGLE_SPEED     = 60.0;  // Matched to 60% so both toggle motors spin evenly without slipping
const double MANUAL_TOGGLE_SPEED   = 60.0;  // Run at 60% speed when holding Button A manually
const int    TOGGLE_TIMEOUT_MS     = 2000;  // Maximum time (in ms) allowed for one auto-toggle attempt
const int    TOGGLE_COOLDOWN_MS    = 50;    // Short delay (in ms) before re-triggering during endgame battles



// Section 3: Drivetrain, Lift, Acceleration Ramp, and Anti-Tip Settings
const int    DEADBAND              = 5;     // Ignores small joystick drift under 5%
const double DRIVE_CURVE           = 0.75;  // Blends linear and cubic input for smoother slow driving
const double TURN_CURVE            = 0.55;  // Curve weight for turning sensitivity
const double MAX_DRIVE_SPEED       = 75.0;  // Maximum forward/reverse drive velocity (%)
const double MAX_TURN_SPEED        = 50.0;  // Maximum turning velocity (%)
const double DRIVE_ACCEL_STEP      = 4.0;   // Max forward speed change (%) per 20 ms loop to eliminate jitter
const double TURN_ACCEL_STEP       = 4.0;   // Max turn speed change (%) per 20 ms loop to eliminate jitter
const double LIFT_SPEED            = 50.0;  // Lowered DR4B lift velocity (%)

const double TILT_MAX              = 12.0;  // Tilt angle (in degrees) required to trigger anti-tip
const double TILT_RECOVER          = 10.0;  // Angle (in degrees) the robot must settle below before releasing anti-tip
const double UNTILT_SPEED          = 60.0;  // Drive speed (%) used to balance the robot if it starts tipping

// Pneumatic claw solenoid states (swap true and false if open and close are backwards)
const bool   CLAW_OPEN             = false;
const bool   CLAW_CLOSED           = true;



// Helper Functions

// Applies a blended cubic curve to joystick values and scales to maxSpeed
double adaptiveCurve(double rawInput, double curveWeight, double maxSpeed) {
  if (std::abs(rawInput) < DEADBAND) {
    return 0.0;
  }
  double norm = rawInput / 100.0;
  double curved = ((1.0 - curveWeight) * norm) + (curveWeight * std::pow(norm, 3));
  return curved * maxSpeed;
}

// Gradually ramps currentSpeed toward targetSpeed by maxStep per loop to stop jerky motion
double slewLimit(double currentSpeed, double targetSpeed, double maxStep) {
  double delta = targetSpeed - currentSpeed;
  if (delta > maxStep) {
    return currentSpeed + maxStep;
  }
  if (delta < -maxStep) {
    return currentSpeed - maxStep;
  }
  return targetSpeed;
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

// Checks if the toggle in front of the sensor is ONLY yellow or the opposite alliance color
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

// Controls both toggle motors using manual Button A override or strict yellow/opponent color sensing
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

  // Start an automatic flip only when yellow or the opposing alliance color is detected
  if (!autoToggleRunning) {
    if ((Brain.Timer.system() - lastAutoToggleEnd >= TOGGLE_COOLDOWN_MS) &&
        backToggleNeedsFlip()) {
      autoToggleRunning = true;
      autoToggleStart = Brain.Timer.system();
    }
  }

  // Spin ONLY while yellow or the opposite alliance color remains in range and within timeout
  if (autoToggleRunning) {
    bool timedOut = (Brain.Timer.system() - autoToggleStart >= TOGGLE_TIMEOUT_MS);

    if (backToggleNeedsFlip() && !timedOut) {
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



// 180-Degree Rotated UI Drawing Helpers (for upside-down mounted V5 Brain)

// Draws a rectangle using upside-down coordinates (top-left of your view -> screen bottom-right)
void drawRect180(int viewX, int viewY, int width, int height) {
  Brain.Screen.drawRectangle(480 - viewX - width, 240 - viewY - height, width, height);
}

// Returns a 5-row (3-bit wide) bitmap for each character so text itself renders upside down
void getGlyph3x5(char c, uint8_t rows[5]) {
  switch (c) {
    case '0': rows[0]=0b111; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case '1': rows[0]=0b010; rows[1]=0b110; rows[2]=0b010; rows[3]=0b010; rows[4]=0b111; break;
    case '2': rows[0]=0b111; rows[1]=0b001; rows[2]=0b111; rows[3]=0b100; rows[4]=0b111; break;
    case '3': rows[0]=0b111; rows[1]=0b001; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case '4': rows[0]=0b101; rows[1]=0b101; rows[2]=0b111; rows[3]=0b001; rows[4]=0b001; break;
    case '5': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case '6': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b101; rows[4]=0b111; break;
    case '7': rows[0]=0b111; rows[1]=0b001; rows[2]=0b010; rows[3]=0b010; rows[4]=0b010; break;
    case '8': rows[0]=0b111; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b111; break;
    case '9': rows[0]=0b111; rows[1]=0b101; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case 'A': rows[0]=0b010; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'B': rows[0]=0b110; rows[1]=0b101; rows[2]=0b110; rows[3]=0b101; rows[4]=0b110; break;
    case 'C': rows[0]=0b111; rows[1]=0b100; rows[2]=0b100; rows[3]=0b100; rows[4]=0b111; break;
    case 'D': rows[0]=0b110; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b110; break;
    case 'E': rows[0]=0b111; rows[1]=0b100; rows[2]=0b110; rows[3]=0b100; rows[4]=0b111; break;
    case 'F': rows[0]=0b111; rows[1]=0b100; rows[2]=0b110; rows[3]=0b100; rows[4]=0b100; break;
    case 'G': rows[0]=0b111; rows[1]=0b100; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case 'H': rows[0]=0b101; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'I': rows[0]=0b111; rows[1]=0b010; rows[2]=0b010; rows[3]=0b010; rows[4]=0b111; break;
    case 'L': rows[0]=0b100; rows[1]=0b100; rows[2]=0b100; rows[3]=0b100; rows[4]=0b111; break;
    case 'M': rows[0]=0b101; rows[1]=0b111; rows[2]=0b101; rows[3]=0b101; rows[4]=0b101; break;
    case 'N': rows[0]=0b101; rows[1]=0b111; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'O': rows[0]=0b111; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case 'P': rows[0]=0b111; rows[1]=0b101; rows[2]=0b111; rows[3]=0b100; rows[4]=0b100; break;
    case 'R': rows[0]=0b110; rows[1]=0b101; rows[2]=0b110; rows[3]=0b101; rows[4]=0b101; break;
    case 'S': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case 'T': rows[0]=0b111; rows[1]=0b010; rows[2]=0b010; rows[3]=0b010; rows[4]=0b010; break;
    case 'U': rows[0]=0b101; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case 'V': rows[0]=0b101; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b010; break;
    case 'Y': rows[0]=0b101; rows[1]=0b101; rows[2]=0b010; rows[3]=0b010; rows[4]=0b010; break;
    case ':': rows[0]=0b000; rows[1]=0b010; rows[2]=0b000; rows[3]=0b010; rows[4]=0b000; break;
    case '-': rows[0]=0b000; rows[1]=0b000; rows[2]=0b111; rows[3]=0b000; rows[4]=0b000; break;
    case '/': rows[0]=0b001; rows[1]=0b001; rows[2]=0b010; rows[3]=0b100; rows[4]=0b100; break;
    case '%': rows[0]=0b101; rows[1]=0b001; rows[2]=0b010; rows[3]=0b100; rows[4]=0b101; break;
    default:  rows[0]=0;     rows[1]=0;     rows[2]=0;     rows[3]=0;     rows[4]=0;     break;
  }
}

// Draws crisp 180-degree rotated block text so letters read right-side up when the Brain is upside down
void printText180(int viewX, int viewY, const char* text, int scale, color textColor) {
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setPenColor(textColor);
  Brain.Screen.setFillColor(textColor);

  int len = std::strlen(text);
  for (int i = 0; i < len; i++) {
    uint8_t rows[5];
    getGlyph3x5(text[i], rows);
    int charStartX = viewX + i * (4 * scale);

    for (int r = 0; r < 5; r++) {
      for (int col = 0; col < 3; col++) {
        if (rows[r] & (1 << (2 - col))) {
          drawRect180(charStartX + col * scale, viewY + r * scale, scale, scale);
        }
      }
    }
  }
}

// Draws the 180-degree rotated 2474G dashboard header, panels, and alliance card
void initBrainUI() {
  color allianceColor = (MY_ALLIANCE == ALLIANCE_RED) ? color(210, 30, 45) : color(20, 105, 235);
  color darkPanel     = color(22, 26, 36);
  color borderGray    = color(75, 85, 105);

  Brain.Screen.clearScreen(color(12, 14, 20));

  // Top Header Banner (appears at the top when Brain is mounted upside down)
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(allianceColor);
  drawRect180(0, 0, 480, 42);
  printText180(16, 11, "2474G // OVERRIDE", 4, color::white);

  // Left Telemetry Panel
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(borderGray);
  Brain.Screen.setFillColor(darkPanel);
  drawRect180(10, 52, 294, 178);
  printText180(22, 62, "SYSTEM TELEMETRY", 2, color(160, 175, 200));

  // Right Alliance Color Card
  Brain.Screen.setPenColor(borderGray);
  Brain.Screen.setFillColor(darkPanel);
  drawRect180(314, 52, 156, 178);

  // Solid Alliance Color Square with 2474G inside
  Brain.Screen.setPenWidth(2);
  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFillColor(allianceColor);
  drawRect180(339, 64, 106, 106);
  printText180(353, 109, "2474G", 4, color::white);

  // Label below the alliance color square
  printText180(336, 182, "ALLIANCE COLOR", 2, color::white);
  printText180(376, 202, (MY_ALLIANCE == ALLIANCE_RED) ? "RED" : "BLUE", 3, allianceColor);
}

// Updates live battery, motor temps, tilt, optical sensor, and toggle readings (180-deg rotated)
void updateBrainUI(bool togglesActive) {
  color darkPanel = color(22, 26, 36);

  int battPercent = static_cast<int>(Brain.Battery.capacity());
  if (battPercent > 100) battPercent = 100;
  if (battPercent < 0) battPercent = 0;

  int maxDriveTemp = static_cast<int>(std::max({
    FrontLeft.temperature(percent),
    BackLeft.temperature(percent),
    FrontRight.temperature(percent),
    BackRight.temperature(percent)
  }));

  int maxLiftTemp = static_cast<int>(std::max(
    LiftLeft.temperature(percent),
    LiftRight.temperature(percent)
  ));

  int tiltVal = static_cast<int>(std::round(
    (std::abs(InertialSensor.pitch(degrees)) >= std::abs(InertialSensor.roll(degrees)))
      ? InertialSensor.pitch(degrees)
      : InertialSensor.roll(degrees)
  ));

  int hueVal = static_cast<int>(std::round(OpticalBack.hue()));

  // Clear telemetry rows before redrawing updated numbers
  Brain.Screen.setPenWidth(0);
  Brain.Screen.setFillColor(darkPanel);
  drawRect180(18, 80, 278, 142);

  // 1. Battery Percentage & Charge Bar
  char battBuf[24];
  snprintf(battBuf, sizeof(battBuf), "BAT: %d%%", battPercent);
  printText180(22, 84, battBuf, 3, color::white);

  Brain.Screen.setPenWidth(1);
  Brain.Screen.setPenColor(color(110, 120, 140));
  Brain.Screen.setFillColor(color(12, 14, 20));
  drawRect180(22, 106, 268, 14);

  color battColor = color(45, 210, 95);
  if (battPercent <= 25) {
    battColor = color(230, 50, 50);
  } else if (battPercent <= 50) {
    battColor = color(240, 195, 40);
  }

  int fillWidth = (264 * battPercent) / 100;
  if (fillWidth > 0) {
    Brain.Screen.setPenWidth(0);
    Brain.Screen.setFillColor(battColor);
    drawRect180(24, 108, fillWidth, 10);
  }

  // 2. Motor Temps, Tilt Angle, Optical Hue & Toggle Status
  char tempBuf[32];
  snprintf(tempBuf, sizeof(tempBuf), "DRV:%d%%  LIFT:%d%%", maxDriveTemp, maxLiftTemp);
  printText180(22, 130, tempBuf, 2, color::white);

  char tiltBuf[32];
  snprintf(tiltBuf, sizeof(tiltBuf), "TILT: %d DEG", tiltVal);
  printText180(22, 152, tiltBuf, 2, color::white);

  char optBuf[32];
  snprintf(optBuf, sizeof(optBuf), "OPT: %s H:%d", isToggleInRange() ? "NEAR" : "FAR", hueVal);
  printText180(22, 174, optBuf, 2, color::white);

  printText180(22, 198, togglesActive ? "TOGGLES: SPINNING" : "TOGGLES: HOLDING",
               2, togglesActive ? color(45, 210, 95) : color(175, 185, 205));
}

// Writes alliance, battery, and toggle reminder to the V5 Controller LCD with proper radio pacing
void updateControllerScreen() {
  if (!ControllerMain.installed()) {
    return;
  }
  ControllerMain.Screen.clearScreen();
  wait(60, msec);
  ControllerMain.Screen.setCursor(1, 1);
  ControllerMain.Screen.print("Alliance: %s", (MY_ALLIANCE == ALLIANCE_RED) ? "RED" : "BLUE");
  wait(60, msec);
  ControllerMain.Screen.setCursor(2, 1);
  ControllerMain.Screen.print("Bat: %d%%", static_cast<int>(Brain.Battery.capacity()));
  wait(60, msec);
  ControllerMain.Screen.setCursor(3, 1);
  ControllerMain.Screen.print("A TO TOGGLE JESTIN!");
}



// Competition Functions

void pre_auton(void) {
  vexcodeInit();
  openClaw();

  // Turn on the rear optical sensor LED at 100% brightness
  OpticalBack.setLightPower(100, percent);
  OpticalBack.setLight(ledState::on);

  // Give the V5 wireless radio time to connect before writing to the controller screen
  wait(250, msec);
  updateControllerScreen();

  // Render the 180-degree rotated 2474G V5 Brain dashboard immediately on startup
  initBrainUI();
  updateBrainUI(false);
}

void autonomous(void) {
  // Autonomous routine will be added here
}

void usercontrol(void) {
  // Assign R1 to open the claw and R2 to close it
  ControllerMain.ButtonR1.pressed(openClaw);
  ControllerMain.ButtonR2.pressed(closeClaw);

  // Re-send controller screen text once driver control starts so VEXos does not clear it
  updateControllerScreen();

  initBrainUI();
  int uiLoopCounter = 0;

  // Track smoothed speeds and anti-tip state to prevent jitter
  double currentYPower = 0.0;
  double currentXPower = 0.0;
  int antiTipState = 0; // 0 = level, 1 = tilted positive, -1 = tilted negative

  while (true) {


    // 1. Run manual and automatic rear toggle controls
    bool togglesActive = handleToggleSystem();


    // 2. Read joysticks, apply response curves, and gradually ramp speed to remove jitter
    double rawForward = ControllerMain.Axis3.position();
    double rawTurn    = ControllerMain.Axis1.position();

    double targetYPower = adaptiveCurve(rawForward, DRIVE_CURVE, MAX_DRIVE_SPEED);
    double targetXPower = adaptiveCurve(rawTurn, TURN_CURVE, MAX_TURN_SPEED);

    currentYPower = slewLimit(currentYPower, targetYPower, DRIVE_ACCEL_STEP);
    currentXPower = slewLimit(currentXPower, targetXPower, TURN_ACCEL_STEP);


    // 3. Check inertial sensor pitch and roll with hysteresis so it doesn't jitter at the threshold
    double robotPitch = InertialSensor.pitch(degrees);
    double robotRoll  = InertialSensor.roll(degrees);
    double activeTilt = (std::abs(robotPitch) >= std::abs(robotRoll)) ? robotPitch : robotRoll;

    if (activeTilt > TILT_MAX) {
      antiTipState = 1;
    } else if (activeTilt < -TILT_MAX) {
      antiTipState = -1;
    } else if (std::abs(activeTilt) < TILT_RECOVER) {
      antiTipState = 0;
    }

    if (antiTipState == 1) {
      currentYPower = -UNTILT_SPEED;
      currentXPower = 0.0;
    } else if (antiTipState == -1) {
      currentYPower = UNTILT_SPEED;
      currentXPower = 0.0;
    }

    setDrivePower(currentYPower, currentXPower);


    // 4. Control the DR4B lift at 50% velocity (L1 = up, L2 = down with bottom limit check, hold when released)
    bool raiseLift = ControllerMain.ButtonL1.pressing();
    bool lowerLift = ControllerMain.ButtonL2.pressing();

    if (raiseLift && !lowerLift) {
      LiftLeft.spin(forward, LIFT_SPEED, percent);
      LiftRight.spin(forward, LIFT_SPEED, percent);
    } else if (lowerLift && !raiseLift &&
               LiftLeft.position(degrees) > 0 &&
               LiftRight.position(degrees) > 0) {
      LiftLeft.spin(reverse, LIFT_SPEED, percent);
      LiftRight.spin(reverse, LIFT_SPEED, percent);
    } else {
      LiftLeft.stop(hold);
      LiftRight.stop(hold);
    }


    // 5. Refresh the custom 2474G Brain dashboard every 100 ms
    uiLoopCounter++;
    if (uiLoopCounter >= 5) {
      updateBrainUI(togglesActive);
      uiLoopCounter = 0;
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
