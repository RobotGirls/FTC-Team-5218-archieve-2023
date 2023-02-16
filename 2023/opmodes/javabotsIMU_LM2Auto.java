/*
Copyright (c) September 2017 FTC Teams 25/5218

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package opmodes;

import static opmodes.IMUDriveAutoTest.TURN_SPEED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.DeadReckonTaskWithIMU;
import team25core.FourWheelDirectDrivetrain;
import team25core.FourWheelDirectIMUDrivetrain;
import team25core.GyroTask;
import team25core.IMUGyroDriveTask;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
import team25core.vision.apriltags.AprilTagDetectionTask;


@Autonomous(name = "javabotsIMU_LM2Auto3")
//@Disabled
@Config
public class javabotsIMU_LM2Auto extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectIMUDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private boolean goingToLowerJunction = true;

    private Servo coneServo;
    private Servo armServo;

    private static final double CONE_GRAB = 0.12;
    private static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = .875;
    private static final double ARM_BACK = 0.0918;

    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    private DeadReckonPath initialConeGrabPath;
    private DeadReckonPath depositInFirstGroundJunctionPath;

    static final int SIGNAL_LEFT = 5;
    static final int SIGNAL_MIDDLE = 2;
    static final int SIGNAL_RIGHT = 18;
    static final double FORWARD_DISTANCE = 6;
    static final double DRIVE_SPEED = -0.5;

    DeadReckonPath driveToGround1Path;
    DeadReckonPath driveFromGround1Path;
    DeadReckonPath liftToSmallJunctionPath;
    DeadReckonPath dropToSmallJunctionPath;
    DeadReckonPath secondPath;
    DeadReckonPath lowerLiftToGroundJunctionPath;
    DeadReckonPath raiseLiftOffGroundJunctionPath;

    // Telemetry for IMU from JavaTeleopIMU 1/2/23
//    Telemetry.Item imuStatus;
//    Telemetry.Item imuCalib;
//    Telemetry.Item imuHeading;
//    Telemetry.Item imuRoll;
//    Telemetry.Item imuPitch;
//    Telemetry.Item imuGrav;

    // added for IMU 1/2/23
    private BNO055IMU imu;
    private Telemetry.Item gyroItemTlm; 
    private DeadReckonTaskWithIMU gyroTask;

    private int i = 0;

    // added for IMU 1/3/23
    private boolean debug = false;
    private Telemetry.Item headingTlm;

    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;

    private Telemetry.Item whereAmI;


    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 0.0;
    private boolean showHeading = true;

    Telemetry myTelemetry;
    //= this.telemetry

    /*
     * The default event handler for the robot.
     */

   @Override
    public void handleEvent(RobotEvent e)
    {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }
    }

//    public void startUsingIMU()
//    {
//        myTelemetry = this.telemetry;
//        // Mapping to the IMU which is needed to instantiate it in gyroTask
//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        // instantiating gyroTask
//        gyroTask = new IMUGyroDriveTask(this, imu, 0, true, headingTlm) {
//            @Override
//            public void handleEvent (RobotEvent event) {
//                i = i++;
//                whereAmI.setValue("in IMUGyroDriveTask handleEvent" + i);
//               // gyroTask.displayTelemetry(myTelemetry);
//                if(((IMUGyroEvent) event).kind == EventKind.HIT_TARGET) {
//                    drivetrain.stop();
//                    driveToSignalZone(middlePath);
//                   if (debug) {
//                       whereAmI.setValue("handleGyroEvent:handleEvent:Hit Target");
//                   }
//                } else if (((IMUGyroEvent) event).kind == EventKind.PAST_TARGET) {
//                    drivetrain.turn(TURN_SPEED / 2);
//                   if (debug) {
//                       whereAmI.setValue("handleGyroEvent:handleEvent:Past Target");
//                   }
//                }
//            }
//
//        };
//        // Now that gyroTask is instantiated we can call its init method
//        gyroTask.init();
//        gyroTask.initTelemetry(this.telemetry);
//        gyroTask.setDrivetrain(drivetrain);
//    }

    public void setAprilTagDetection() {
        if (debug) {
            whereAmI.setValue("before detectionTask");
        }
        whereAmI.setValue("before detectionTask");
        detectionTask = new AprilTagDetectionTask(this, "Webcam 1") {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                tagObject = event.tagObject;
                if (debug) {
                    tagIdTlm.setValue(tagObject.id);
                }
                whereAmI.setValue("in handleEvent");
                detectedAprilTagID = tagObject.id;
            }
        };
        whereAmI.setValue("setAprilTagDetection");
        detectionTask.init(telemetry, hardwareMap);
    }

    public void liftToFirstGroundJunction()
    {
        this.addTask(new DeadReckonTask(this, liftToSmallJunctionPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToGroundJunction");
                    //delay(1);
                 driveToFromFirstGroundJunction(driveToGround1Path);
                }
            }
        });
    }

    private void decideWhichSignalWasSeen()
    {

        if (detectedAprilTagID == SIGNAL_LEFT) {
            driveToSignalZone(leftPath);
        } else if (detectedAprilTagID == SIGNAL_MIDDLE) {
            driveToSignalZone(middlePath);
        } else {
            driveToSignalZone(rightPath);
        }
    }

    private void driveToFromFirstGroundJunction(DeadReckonPath driveToGround1Path)
    {
        whereAmI.setValue("in driveToFromFirstGroundJunction");
        RobotLog.i("drives to First Ground Junction");

        gyroTask = new DeadReckonTaskWithIMU(this, driveToGround1Path, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // FIXME temporarily comment this out to test IMU
//                if (path.kind == EventKind.PATH_DONE)
//                {
//                    if (goingToLowerJunction) {
//                        lowerLiftToFirstGroundJunction();
//                        goingToLowerJunction = false;
//                    } else {
//                        // have finished backing away from lower junction
//                        decideWhichSignalWasSeen();
//                    }
//                    //delayAndLowerLift(2);
//                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
    }



    public void lowerLiftToFirstGroundJunction()
    {

        this.addTask(new DeadReckonTask(this, lowerLiftToGroundJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToGroundJunction");

                    coneServo.setPosition(CONE_RELEASE);
                    raiseLiftOffFirstGroundJunction();
                }
            }
        });
    }
    public void raiseLiftOffFirstGroundJunction()
    {
        this.addTask(new DeadReckonTask(this, raiseLiftOffGroundJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToGroundJunction");

                    driveToFromFirstGroundJunction(driveFromGround1Path);

                }
            }
        });
    }

    public void driveToSignalZone(DeadReckonPath signalPath)
    {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTaskWithIMU(this, signalPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");

                }
            }
        });
    }

    public void delayAndLowerLift(int milliseconds)
    {
        addTask(new SingleShotTimerTask(this, 1000*milliseconds) {
            @Override
            public void handleEvent (RobotEvent e) {
                        lowerLiftToFirstGroundJunction();
            }
        });
    }

    public void initPaths()
    {
        driveToGround1Path = new DeadReckonPath();
        driveFromGround1Path = new DeadReckonPath();
        liftToSmallJunctionPath = new DeadReckonPath();
        lowerLiftToGroundJunctionPath = new DeadReckonPath();
        raiseLiftOffGroundJunctionPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath= new DeadReckonPath();

        driveToGround1Path.stop();
        driveFromGround1Path.stop();
        liftToSmallJunctionPath.stop();
        lowerLiftToGroundJunctionPath.stop();
        raiseLiftOffGroundJunctionPath.stop();
        leftPath.stop();
        middlePath.stop();
        rightPath.stop();

        // drives to first ground junction
        // strafe to left align with ground junction
        // FIXME This is just getting temporarily commented out for IMU
        // initial distance must be 9
//        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,18, 0.5);
          driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,  24, 0.5);
//          driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, -0.5);
//          driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, 0.5);
//          driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10, -0.5);
        // FIXME Temporarily comment this out to test out the IMU
//        // back to align with wall since strafe drifts
//        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,2, -0.5);
//        // drive up to ground junction
//        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.5);

//        // back away from low junction
//        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,8, -0.5);
//        // strife to the left
//        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,9, -0.5);
//        // back up to align with the wall
//        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,3, -0.5);


        // lifts to small junction
        liftToSmallJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.7);


        // lowers lift to the Ground Junction
        lowerLiftToGroundJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.6);

        // raise lift off the Ground Junction
        raiseLiftOffGroundJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, 0.65);


        // Based on Signal ID:
        // return to initial to go forward then to the left

//         // strife to left
//        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,18, -0.5);
//        // go back to align with the wall
//        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
//        // go straignt
//        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
//
//        // return to initial then go forward
//        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
//        ;
//        // return to initial then go forward then right
//        // straife to right
//        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,16, 0.5);
//        // go back align with wall
//        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, -0.5);
//       // go straight
//        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
    }

    @Override
    public void init()
    {
        // cindy added
        super.init();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        armServo = hardwareMap.servo.get("armServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectIMUDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();
        // We are setting the target yaw to the value of the constant
        // TARGET_YAW_FOR_DRIVING_STRAIGHT which is zero for our robot to
        // go straight.
        drivetrain.setTargetYaw(TARGET_YAW_FOR_DRIVING_STRAIGHT);

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        whereAmI = telemetry.addData("location in code", "init");

        if (debug) {
            tagIdTlm = telemetry.addData("tagId", "none");
            gyroItemTlm = telemetry.addData("Gyro state:", "Not at target");
        }

        // initialized heading telemetry
        headingTlm = telemetry.addData("Current/target heading is: ", "0.0");


        // the following tlm is used when debug is true
       // tagIdTlm = telemetry.addData("tagId","none");

        initPaths();

        // startUsingIMU();
    }

    @Override
    public void start()
    {
        // FIXME For IMU temporarily starting from driveToFromFirstGroundJunction
       // liftToFirstGroundJunction();

        // added addTask
       // addTask(gyroTask);
        if (debug){
            whereAmI.setValue("in Start");
        }
        driveToFromFirstGroundJunction(driveToGround1Path);
        whereAmI.setValue("in Start");
         setAprilTagDetection();
         addTask(detectionTask);
    }
}

