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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import team25core.DeadmanMotorTask;
import team25core.GamepadTask;
import team25core.MechanumGearedDrivetrain;
import team25core.RobotEvent;
import team25core.StandardFourMotorRobot;
import team25core.TankMechanumControlSchemeReverse;
import team25core.TeleopDriveTask;

@TeleOp(name = "JavaTeleopIMU")
//@Disabled
public class JavaTeleopIMU extends StandardFourMotorRobot {


    private TeleopDriveTask drivetask;

    private enum Direction {
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }
    //added field centric

    private Telemetry.Item buttonTlm;
    private Telemetry.Item locationTlm;

    private static final double CONE_GRAB = 0.2;
    private static final double CONE_RELEASE = 0.67;

    private static final double ARM_FRONT = 0.06;
    private static final double ARM_BACK = 1;

    private BNO055IMU imu;

    //new
    private Orientation angles;
    private Acceleration gravity;

    private DcMotor liftMotor;
    // private DcMotor intakeMotor;

    private Servo coneServo;
    private Servo armServo;

    private boolean currentlySlow = false;

    private DeadmanMotorTask liftMotorUpTask;
    private DeadmanMotorTask liftMotorDownTask;

//    private DeadmanMotorTask intakeTask;
//    private DeadmanMotorTask outtakeTask;

    MecanumFieldCentricDriveScheme scheme;

    private MechanumGearedDrivetrain drivetrain;

    private static final int TICKS_PER_INCH = 79;

    Telemetry.Item imuStatus;
    Telemetry.Item imuCalib;
    Telemetry.Item imuHeading;
    Telemetry.Item imuRoll;
    Telemetry.Item imuPitch;
    Telemetry.Item imuGrav;

    @Override
    public void handleEvent(RobotEvent e) {
    }

    @Override
    public void init() {

        super.init();
        initIMU();

        scheme = new MecanumFieldCentricDriveScheme(gamepad1,imu, this.telemetry);
        scheme.initTelemetry(imuStatus, imuCalib, imuHeading, imuRoll, imuPitch, imuGrav);
        scheme.setCanonical(MecanumFieldCentricDriveScheme.MotorDirection.NONCANONICAL);
        //mechanisms
//        carouselMech = hardwareMap.get(DcMotor.class, "carouselMech");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        armServo = hardwareMap.servo.get("armServo");

        // using encoders to record ticks
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        coneServo.setPosition(0.4);
        armServo.setPosition(0.06);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telemetry
        buttonTlm = telemetry.addData("buttonState", "unknown");
        locationTlm = telemetry.addData("location","init");

//        TankMechanumControlSchemeReverse scheme = new TankMechanumControlSchemeReverse(gamepad1);
        drivetrain = new MechanumGearedDrivetrain(motorMap);
//        drivetrain.setCanonicalMotorDirection();
        // Note we are swapping the rights and lefts in the arguments below
        // since the gamesticks were switched for some reason and we need to do
        // more investigation

        drivetask = new TeleopDriveTask(this, scheme, backLeft, backRight, frontLeft, frontRight);

        liftMotorUpTask = new DeadmanMotorTask(this, liftMotor,  -1.0, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_STICK_UP);
        liftMotorDownTask = new DeadmanMotorTask(this, liftMotor, 1.0, GamepadTask.GamepadNumber.GAMEPAD_2, DeadmanMotorTask.DeadmanButton.LEFT_STICK_DOWN);
    }

    public void initIMU()
    {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //changed back to Radians
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;//New
        // Without this, data retrieving from the IMU throws an exception
        parameters.mode = BNO055IMU.SensorMode.IMU;//New
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.loggingTag           = "IMU";

        imu.initialize(parameters);

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();

        imuStatus = telemetry.addData("Status", imu.getSystemStatus().toString());
        imuCalib = telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        imuHeading = telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        imuRoll = telemetry.addData("Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        imuPitch = telemetry.addData("Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        imuGrav = telemetry.addData("Grav", gravity.toString());
//        telemetry.update();
        telemetry.setMsTransmissionInterval(100);


    }

    //New
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    @Override
    public void start() {

        //Gamepad 1
        this.addTask(drivetask);
        locationTlm.setValue("in start");

        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_1) {
            //@Override
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;
                locationTlm.setValue("in gamepad1 handler");
                switch (gamepadEvent.kind) {
                    case BUTTON_X_DOWN:
                        // If slow, then normal speed. If fast, then slow speed of motors.
                        //pertains to slowmode
                        if (currentlySlow) {
                            drivetask.slowDown(0.7);
                            currentlySlow = false;
                        } else {
                            drivetask.slowDown(0.3);
                            currentlySlow = true;
                        }
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                }
            }
        });

        //Gamepad 2
        this.addTask(liftMotorUpTask);
        this.addTask(liftMotorDownTask);
//        this.addTask(intakeTask);
//        this.addTask(outtakeTask);
        this.addTask(new GamepadTask(this, GamepadTask.GamepadNumber.GAMEPAD_2) {
            //@Override
            public void handleEvent(RobotEvent e) {
                GamepadEvent gamepadEvent = (GamepadEvent) e;
                locationTlm.setValue("in gamepad2 handler");
                switch (gamepadEvent.kind) {

                    case BUTTON_X_DOWN:
                        //position 0
                        armServo.setPosition(ARM_FRONT);
                        break;
                    case BUTTON_B_DOWN:
                        //position 1
                        armServo.setPosition(ARM_BACK);
                        break;
                    case BUTTON_A_DOWN:
                        //position 1
                        coneServo.setPosition(CONE_GRAB);
                        break;
                    case BUTTON_Y_DOWN:
                        //position 0 (original pos)
                        coneServo.setPosition(CONE_RELEASE);
                        break;
                    default:
                        buttonTlm.setValue("Not Moving");
                        break;
                    case LEFT_STICK_UP:

                }
            }
        });
    }
}

