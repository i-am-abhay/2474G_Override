#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller ControllerMain = controller(primary);
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT1, ratio6_1, false);
motor BackLeft = motor(PORT9, ratio6_1, false);
motor FrontRight = motor(PORT2, ratio6_1, false);
motor BackRight = motor(PORT10, ratio6_1, false);
motor ClawMotor = motor(PORT6, ratio18_1, false);
inertial InertialSensor = inertial(PORT5);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
