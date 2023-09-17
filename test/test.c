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
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <math.h>

#define MAXCHAR 256

char row[MAXCHAR];

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

int readCSV(FILE* file_ptr, double *data)
{

	fgets(row, MAXCHAR, file_ptr);
    if(feof(file_ptr))   
        return 1;
    
    // printf("Row is: %s", row);
	data[0] = strtod(strtok(row, ","),NULL);
	// printf("%lf\n",data[0]);
	for(int i =1; i<6; i++)
	{
		data[i] = strtod(strtok(NULL,","),NULL);
		// printf("%lf\n",data[i]);
	}
	return 0;
}


int main(int argc, char **argv) 
{
	wb_robot_init();
	time_step = (int)wb_robot_get_basic_time_step();
	wb_keyboard_enable(time_step);

	WbDeviceTag motors[7];
	WbDeviceTag gripper[3];
	motors[1] = wb_robot_get_device("joint_1");
	motors[2] = wb_robot_get_device("joint_2");
	motors[3] = wb_robot_get_device("joint_3");
	motors[4] = wb_robot_get_device("joint_4");
	motors[5] = wb_robot_get_device("joint_5");
	motors[6] = wb_robot_get_device("joint_6");
	gripper[1] = wb_robot_get_device("joint_base_to_jaw_1");
	gripper[2] = wb_robot_get_device("joint_base_to_jaw_2");



	// set the motor velocity
	// first we make sure that every joint is at its initial position
	wb_motor_set_position(motors[1], 0.0);
	wb_motor_set_position(motors[2], 0.0);
	wb_motor_set_position(motors[3], 0.0);
	wb_motor_set_position(motors[4], 0.0);
	wb_motor_set_position(motors[5], 0.0);
	wb_motor_set_position(motors[6], 0.0);
	wb_motor_set_position(gripper[1], 0.009);
	wb_motor_set_position(gripper[2], 0.009);

	// set the motors speed. Here we set it to 1 radian/second
	wb_motor_set_velocity(motors[1], 1.0);
	wb_motor_set_velocity(motors[2], 0.7);
	wb_motor_set_velocity(motors[3], 1.0);
	wb_motor_set_velocity(motors[4], 1.0);
	wb_motor_set_velocity(motors[5], 1.0);
	wb_motor_set_velocity(motors[6], 1.0);
	wb_motor_set_velocity(gripper[1], 1.0);
	wb_motor_set_velocity(gripper[2], 1.0);

	double angles[6];
	// control with keyboard
	FILE* stream = fopen("solutions.csv", "r");
	if (stream == NULL) {
		perror("Failed to open file");
		return 1;
	}
	else{
        printf("File opened successfuly");
	    fgets(row, MAXCHAR, stream);
    }



	passive_wait(2);
           unsigned int loop = 0;
	while (wb_robot_step(time_step) != -1)
	{
		
		if(readCSV(stream,angles))
			break;
		
		wb_motor_set_position(motors[1],angles[0]);//+
		wb_motor_set_position(motors[2],-1*angles[1]);//-
		wb_motor_set_position(motors[3],-1*angles[2]);//-
		wb_motor_set_position(motors[4],-1*angles[3]);//-
		wb_motor_set_position(motors[5],angles[4]);//+
		wb_motor_set_position(motors[6],angles[5]);//+	
                      if(loop ==0)
          		     passive_wait(5);
        		loop++;
		
	}

	wb_robot_cleanup();
	return 0;
}
