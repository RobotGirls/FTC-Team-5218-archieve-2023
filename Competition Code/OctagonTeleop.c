#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     lightSensor,    sensorI2CHiTechnicCompass)
#pragma config(Sensor, S3,     IRsensor,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S4,     compass,        sensorI2CHiTechnicCompass)
#pragma config(Motor,  motorA,          motorDown,     tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop, encoder)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     rightFront,    tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     elevatorRight, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     leftFront,     tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     elevatorLeft,  tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     leftRear,      tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     rightRear,     tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    shelfServo,           tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    IRservo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    Bumper,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Tele-Operation Mode Code Template
//
// This file contains a template for simplified creation of an tele-op program for an FTC
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "../library/sensors/drivers/hitechnic-sensormux.h"
#include "../library/sensors/drivers/hitechnic-irseeker-v2.h"
#include "../library/sensors/drivers/hitechnic-compass.h"
#include "../library/sensors/drivers/lego-light.h"
#include "DrivetrainOctagon.c"

#define TOPHAT_SPEED 30
#define IRUP 104
#define IRDOWN 19
#define TILTDOWN 119
#define TILTHALF 96
#define TILTUP 69
#define BUMPERDOWN 148
#define BUMPERUP 250
#define ROTATE_SPEED 40

int reversed = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of tele-op mode, you may want to perform some initialization on your robot
// and the variables within your program.
//
// In most cases, you may not have to add any code to this function and it will remain "empty".
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot()
{
    // Place code here to sinitialize servos to starting positions.
    // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.

    allMotorsOff();
    servo[IRservo] = IRDOWN;
    servo[Bumper] = BUMPERUP;
    servo[shelfServo] = TILTUP;

    return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                         Main Task
//
// The following is the main code for the tele-op robot operation. Customize as appropriate for
// your specific robot.
//
// Game controller / joystick information is sent periodically (about every 50 milliseconds) from
// the FMS (Field Management System) to the robot. Most tele-op programs will follow the following
// logic:
//   1. Loop forever repeating the following actions:
//   2. Get the latest game controller / joystick settings that have been received from the PC.
//   3. Perform appropriate actions based on the joystick + buttons settings. This is usually a
//      simple action:
//      *  Joystick values are usually directly translated into power levels for a motor or
//         position of a servo.
//      *  Buttons are usually used to start/stop a motor or cause a servo to move to a specific
//         position.
//   4. Repeat the loop.
//
// Your program needs to continuously loop because you need to continuously respond to changes in
// the game controller settings.
//
// At the end of the tele-op period, the FMS will autonmatically abort (
// stop) execution of the program. You know what, I can do this!
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Octodrivetrain()
{
	int y = joystick.joy1_y1;
	int x = joystick.joy1_x1;

    nxtDisplayCenteredTextLine(4, "Tophat %d", joystick.joy1_TopHat);

	if (joy1Btn(6))
    {
 	    rotateClockwise(ROTATE_SPEED);
    }
    else if (joy1Btn(5))
    {
	    rotateCounterClockwise(ROTATE_SPEED);
    }
    else if (abs(y) > 10 || abs(x) > 10)
 	{
 		motor[leftFront] = ((x+y)/2);
 		motor[rightFront] = ((x-y)/2);
 		motor[leftRear] = ((y-x)/2);
 		motor[rightRear] = ((-y-x)/2);
 	}
    else if (joy1Btn(1))
 	{
        reversed =! reversed;
 	}
    else if (joystick.joy1_TopHat != -1)
    {
        switch(joystick.joy1_TopHat)
        {
            case 0:
                moveForwardOn(TOPHAT_SPEED);
                break;
            case 2:
                if (reversed) {
                    moveSideRightOn(TOPHAT_SPEED);
                } else {
                    moveSideLeftOn(TOPHAT_SPEED);
                }
                break;
            case 4:
                moveBackwardOn(TOPHAT_SPEED);
                break;
            case 6:
               if (reversed) {
                    moveSideLeftOn(TOPHAT_SPEED);
               } else {
                    moveSideRightOn(TOPHAT_SPEED);
               }
               break;
            default:
                break;
        }
    }
    else
    {
        moveForwardOff();
    }
}
void gravitySlide()
{
    if (joy2Btn(5))
	{
		motor[elevatorRight] = 90;
        motor[elevatorLeft] = -90;
        motor[motorDown] = 0;
    }
    else if (joy2Btn(7))
    {
        motor[elevatorRight] = -15;
        motor[elevatorLeft] = 15;
        motor[motorDown] = -85;
    }
    //else if (joystick.joy2_y1 >= 20)
    //{
    //        motor[elevatorLeft] = -10;
    //}
    //else if (joystick.joy2_y1 <= -20)
    //{
    //        motor[elevatorLeft] = 10;
    //}
    //else if (joystick.joy2_y2 >= 20)
    //{
    //        motor[elevatorRight] = 10;
    //}
    //else if (joystick.joy2_y2 <= -20)
    //{
    //        motor[elevatorRight] = -10;
    //}
    else
    {
        motor[elevatorRight] = 0;
        motor[elevatorLeft] = 0;
    }
}

void IRarm()
{
  if (joy2Btn(6))
	{
		servo[IRservo] = IRUP;
    }
    else if (joy2Btn(8))
    {
        servo[IRservo] = IRDOWN;
    }
}

void gravityTilt()
{
     if (joy2Btn(4))
	{
		servo[shelfServo] = TILTDOWN;
    }
    else if (joy2Btn(2))
    {
        servo[shelfServo] = TILTUP;
    }
    else if (joy2Btn(3))
    {
        servo[shelfServo] = TILTHALF;
    }
}
void sakshiBumper()
{
    if (joy1Btn(9))
    {
        servo[Bumper] = BUMPERDOWN;
    }
    else if (joy1Btn(10))
    {
        servo[Bumper] = BUMPERUP;
    }
}

task main()
{
    initializeRobot();

    waitForStart();   // wait for start of tele-op phase

    servo[IRservo] = IRUP;
    while (true)
    {
		getJoystickSettings(joystick);//Get joystick input
		Octodrivetrain();//call the forwarddrivetrain function
        gravitySlide();
        gravityTilt();
        sakshiBumper();
        IRarm();
    }
}
