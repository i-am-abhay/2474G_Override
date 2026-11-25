#pragma once
#include "vex.h"

using namespace vex;



// V5 Smart Port and 3-Wire Port Definitions (facing the front claw)

// Drivetrain motor ports (4 omni wheels, 30:48 gear ratio, 600 RPM blue cartridges)
#define FRONT_LEFT_PORT      PORT2
#define BACK_LEFT_PORT       PORT10
#define FRONT_RIGHT_PORT     PORT1
#define BACK_RIGHT_PORT      PORT9

// DR4B lift motor ports (12T:84T 7:1 gear ratio, 200 RPM green cartridges)
#define LIFT_LEFT_PORT       PORT15
#define LIFT_RIGHT_PORT      PORT3

// Rear toggle motor ports (mounted on the back of the robot)
#define TOGGLE_LEFT_PORT     PORT7
#define TOGGLE_RIGHT_PORT    PORT8

// Sensor ports
#define INERTIAL_PORT        PORT5
#define OPTICAL_BACK_PORT    PORT11

// 3-wire pneumatic solenoid port for the claw
#define CLAW_SOLENOID_PORT   Brain.ThreeWirePort.A



// Global Hardware Declarations

extern brain Brain;
extern controller ControllerMain;

// Drivetrain motors (left and right sides are based on facing the front claw)
extern motor FrontLeft;
extern motor BackLeft;
extern motor FrontRight;
extern motor BackRight;

// DR4B lift motors (left and right sides are based on facing the front claw)
extern motor LiftLeft;
extern motor LiftRight;

// Rear toggle motors (both motors spin together)
extern motor ToggleLeft;
extern motor ToggleRight;

// Sensors and pneumatic claw
extern inertial InertialSensor;
extern optical OpticalBack;
extern digital_out ClawSolenoid;

// Sets up motor brake modes, starting encoder positions, optical LED, and inertial calibration
void vexcodeInit(void);
