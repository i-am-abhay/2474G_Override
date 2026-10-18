/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  const int tilt_max = 5; // the angle of tilt that the robot can be at before it tries to untilt itself.
  const int untiltspeed = 60; // Speed the robot untilts itself (60%)
  // THIS IS MANUAL CONTROL
  while (1) {
    int forwardpower = ControllerMain.Axis3.position(); // Looks at the left joystick, up and down to see how fast to move
    int turnpower = ControllerMain.Axis1.position(); // Looks at the right joystick, left and right to see how fast to turn
    int sidepower = ControllerMain.Axis4.position(); // Looks at the left joystick, left and right to see how fast to strafe.
    double robotpitch = InertialSensor.pitch(degrees); // Measures how much the robot is actually tilting and stores it in the variable robotpitch

    // This is useful because it helps absorb the impact rather than just tipping over when struck. For example, if someone pushes you over, standing still will make
    // you loose balance and fall, but if you took a step back to counteract the force, then you are less likely to fall. Hence, the same logic.
    if (robotpitch > tilt_max) { // Checks if the robot is tilting forward too much
      forwardpower = untiltspeed; // If it is tilting forward too much, then it sets the forwardpower to the untiltspeed to correct itself and counteract by moving forwards.
    }
    else if (robotpitch < -tilt_max) { // Checks if the robot is tilting backward too much
      forwardpower = -untiltspeed; // If it is tilting backward too much, then it sets the forwardpower to the negative untiltspeed to correct itself and counteract by moving backwards.
    }

    int leftMotorSpeed = forwardpower + turnpower;  // Adds turnspeed to forward speed so it speeds up when turning right
    int rightMotorSpeed = forwardpower - turnpower; // Subtracts turnspeed from forward speed so it slows down when turning right
    FrontLeft.spin(reverse, leftMotorSpeed, percent); //  Left motors spins in reverse to make the robot move forward
    BackLeft.spin(reverse, leftMotorSpeed, percent); // Left motors spins in reverse to make the robot move forward
    FrontRight.spin(forward, rightMotorSpeed, percent); // Right motors spins forward to make the robot move forward
    BackRight.spin(forward, rightMotorSpeed, percent); // Right motors spins forward to make the robot move forward

    wait(20, msec);
  }
}

void clawOpen() {
  ClawMotor.setVelocity(100, percent);
  ClawMotor.spinToPosition(90, degrees, true);
}

void clawClose() {
  ClawMotor.setVelocity(100, percent);
  // Open the Claw and hold the position for two seconds
  ClawMotor.spinToPosition(0, degrees, true);
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Claw calibration at start of program
  ClawMotor.setVelocity(30, percent);
  ClawMotor.spin(reverse);
  wait(2, seconds);
  ClawMotor.setPosition(0, degrees);
  
  // Claw open & close 
  Controller1.ButtonR1.pressed(clawClose);
  Controller1.ButtonR2.pressed(clawOpen);

  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
