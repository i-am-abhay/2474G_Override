#pragma once
#include "vex.h"

using namespace vex;



// EASY-CHANGE PORT DEFINITIONS (Perspective: Facing the Front Claw)

// Drivetrain Ports (4-Motor All-Omni, 30:48 ratio, Blue 600 RPM cartridges)
#define FRONT_LEFT_PORT      PORT2
#define BACK_LEFT_PORT       PORT10
#define FRONT_RIGHT_PORT     PORT1
#define BACK_RIGHT_PORT      PORT9

// DR4B Lift Ports (12T:84T 7:1 ratio, Green 200 RPM cartridges)
#define LIFT_LEFT_PORT       PORT4
#define LIFT_RIGHT_PORT      PORT3

// Rear Toggle Motor Ports (Both mounted at the back of the bot)
#define TOGGLE_LEFT_PORT     PORT7
#define TOGGLE_RIGHT_PORT    PORT8

// Sensor Ports
#define INERTIAL_PORT        PORT5
#define OPTICAL_BACK_PORT    PORT11

// Three-Wire Pneumatic Port
#define CLAW_SOLENOID_PORT   Brain.ThreeWirePort.A



// GLOBAL HARDWARE DECLARATIONS

extern brain Brain;
extern controller ControllerMain;

// Drivetrain Motors (Left/Right defined while looking straight at the claw)
extern motor FrontLeft;
extern motor BackLeft;
extern motor FrontRight;
extern motor BackRight;

// DR4B Lift Motors (Left/Right defined while looking straight at the claw)
extern motor LiftLeft;
extern motor LiftRight;

// Rear Toggle Motors (Both run together)
extern motor ToggleLeft;
extern motor ToggleRight;

// Sensors & Pneumatics
extern inertial InertialSensor;
extern optical OpticalBack;
extern digital_out ClawSolenoid;

/**
 * Initializes sensors, optical LED, motor brake modes, and starting encoders.
 */
void vexcodeInit(void);
