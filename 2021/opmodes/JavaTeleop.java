/*
 * Copyright (c) September 2017 FTC Teams 25/5218
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted (subject to the limitations in the disclaimer below) provided that
 *  the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or
 *  other materials provided with the distribution.
 *
 *  Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
 *  endorse or promote products derived from this software without specific prior
 *  written permission.
 *
 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 *  LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadmanMotorTask;
import team25core.GamepadTask;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;
import team25core.SingleShotTimerTask;
import team25core.StandardFourMotorRobot;
import team25core.TankMechanumControlScheme;
import team25core.TankMechanumControlSchemeReverse;
import team25core.TeleopDriveTask;

@TeleOp(name = "JavaTeleop")
//@Disabled
public class JavaTeleop extends StandardFourMotorRobot {


    private Telemetry.Item linearPos;
    private Telemetry.Item linearEncoderVal;

    private TeleopDriveTask drivetask;
    private DcMotor launchMechRight;
    private DcMotor intakeMech;
    private DcMotor conveyorMech;
    private Servo dispenserMech;
    private boolean dispenserMechIsOpen = false;
    private final double OPEN_SERVO = (float) 256.0/256.0;
    private final double CLOSE_SERVO = (float) 128.0/256.0;
   // private DcMotor launchMechLeft;

    //private FourWheelDirectDrivetrain drivetrain;
    //private MechanumGearedDrivetrain drivetrain;

    private static final int TICKS_PER_INCH = 79;

    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {

        super.init();

        //mapping the wheels
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        //mapping the launch mech and intake mech
        launchMechRight = hardwareMap.get(DcMotor.class, "launchMechRight");
        intakeMech = hardwareMap.get(DcMotor.class, "intakeMech");
        conveyorMech = hardwareMap.get(DcMotor.class, "conveyorMech");
        dispenserMech = hardwareMap.servo.get("dispenserMech");
     //   launchMechLeft = hardwareMap.get(DcMotor.class, "launchMechLeft");

        //**CONTINUE FROM HERE**

        // using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMechRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // launchMechLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       /* launch = new OneWheelDirectDrivetrain(launchMech);
        launch.resetEncoders();
        launch.encodersOn();

        intake = new OneWheelDirectDrivetrain(intakeMech);
        intake.resetEncoders();
        intake.encodersOn();

        */

        //TankMechanumControlSchemeReverse scheme = new TankMechanumControlSchemeReverse(gamepad1);

        //code for forward mechanum drivetrain:
        //drivetrain = new MechanumGearedDrivetrain(360, frontRight, rearRight, frontLeft, rearLeft);
    }

    @Override
    public void start() {

        //TankMechanumControlSchemeReverse scheme = new TankMechanumControlSchemeReverse(gamepad1);
        TankMechanumControlScheme scheme = new TankMechanumControlScheme(gamepad1, TankMechanumControlScheme.MotorDirection.NONCANONICAL);

        drivetask = new TeleopDriveTask(this, scheme, frontLeft, frontRight, backLeft, backRight);

        this.addTask(drivetask);

        //gamepad 1
        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            //@Override
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;

                switch (gamepadEvent.kind) {
                    case BUTTON_Y_DOWN:
                        // enable the launch mech
                        launchMechRight.setPower(1);
                       // launchMechLeft.setPower(-1);
                        break;
                    case BUTTON_Y_UP:
                        // stop the launch mech
                        launchMechRight.setPower(0);
                        //launchMechLeft.setPower(0);
                        break;
                    case BUTTON_A_DOWN:
                        // enable the intake mech
                        intakeMech.setPower(-1);
                        break;
                    case BUTTON_A_UP:
                        // stop the intake mech
                        intakeMech.setPower(0);
                        break;
                    case BUTTON_X_DOWN:
                        // enable the conveyor mech
                        conveyorMech.setPower(-1);
                        break;
                    case BUTTON_X_UP:
                        // stop the conveyor mech
                        conveyorMech.setPower(0);
                        break;
                    case BUTTON_B_DOWN:
                        // enable the dispenser mech
                        if(dispenserMechIsOpen) {
                            dispenserMech.setPosition(CLOSE_SERVO);
                            dispenserMechIsOpen = false;
                        } else {
                            dispenserMech.setPosition(OPEN_SERVO);
                            dispenserMechIsOpen = true;
                        }
                        break;
                }
            }
        });
    }
}

