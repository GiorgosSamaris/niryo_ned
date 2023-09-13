/*
 * File:          test.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
static int time_step;

static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do
    step();
  while (start_time + sec > wb_robot_get_time());
}

double degToRad(double deg)
{
	return deg/180*3.1415;
}




int main(int argc, char **argv) {
  wb_robot_init();

  time_step = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(time_step);

  WbDeviceTag motors[7];
  WbDeviceTag gripper;
  motors[1] = wb_robot_get_device("joint_1");
  motors[2] = wb_robot_get_device("joint_2");
  motors[3] = wb_robot_get_device("joint_3");
  motors[4] = wb_robot_get_device("joint_4");
  motors[5] = wb_robot_get_device("joint_5");
  motors[6] = wb_robot_get_device("joint_6");
  gripper = wb_robot_get_device("gripper::left");
  
   

  // set the motor velocity
  // first we make sure that every joint is at its initial position
  wb_motor_set_position(motors[1], 0.0);
  wb_motor_set_position(motors[2], 0.0);
  wb_motor_set_position(motors[3], 0.0);
  wb_motor_set_position(motors[4], 0.0);
  wb_motor_set_position(motors[5], 0.0);
  wb_motor_set_position(motors[6], 0.0);
  wb_motor_set_position(gripper, 0.009);

  // set the motors speed. Here we set it to 1 radian/second
  wb_motor_set_velocity(motors[1], 1.0);
  wb_motor_set_velocity(motors[2], 0.7);
  wb_motor_set_velocity(motors[3], 1.0);
  wb_motor_set_velocity(motors[4], 1.0);
  wb_motor_set_velocity(motors[5], 1.0);
  wb_motor_set_velocity(motors[6], 1.0);
  wb_motor_set_velocity(gripper, 1.0);

  // control with keyboard
  
  passive_wait(3);
  
  while (wb_robot_step(time_step) != -1) {
    wb_motor_set_position(motors[1], 0.70556818);//+
    wb_motor_set_position(motors[3], 0.04358794);//-
    wb_motor_set_position(motors[5], -0.79856532);//+
    wb_motor_set_position(motors[6], -0.86522815);//+
  passive_wait(2);
    wb_motor_set_position(motors[2], 0.84215326);//-
  passive_wait(2);
    wb_motor_set_position(gripper, 0.000);
  passive_wait(1);
   wb_motor_set_position(motors[1], 0.0);
  wb_motor_set_position(motors[2], 0.0);
  wb_motor_set_position(motors[3], 0.0);
  wb_motor_set_position(motors[4], 0.0);
  wb_motor_set_position(motors[5], 0.0);
  wb_motor_set_position(motors[6], 0.0);
  wb_motor_set_position(gripper, 0.009);
    break;

  }

  wb_robot_cleanup();
  return 0;
}
