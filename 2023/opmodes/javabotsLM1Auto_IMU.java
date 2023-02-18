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
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
import team25core.vision.apriltags.AprilTagDetectionTask;


@Autonomous(name = "javabotsLM1IMU_Auto")
//@Disabled
//@Config
public class javabotsLM1Auto_IMU extends Robot {

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

    private Telemetry.Item headingTlm;

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

    private DeadReckonTaskWithIMU gyroTask;
    private BNO055IMU imu;

    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 0.0;
    private boolean showHeading = true;


    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;


    private Telemetry.Item whereAmI;
 
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


    public void setAprilTagDetection() {
        whereAmI.setValue("before detectionTask");
        detectionTask = new AprilTagDetectionTask(this, "Webcam 1") {
            @Override
            public void handleEvent(RobotEvent e) {
                TagDetectionEvent event = (TagDetectionEvent) e;
                tagObject = event.tagObject;
                tagIdTlm.setValue(tagObject.id);
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
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    if (goingToLowerJunction) {
                        lowerLiftToFirstGroundJunction();
                        goingToLowerJunction = false;
                    } else {
                        // have finished backing away from lower junction
                        decideWhichSignalWasSeen();

                    }
                }
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

        gyroTask = new DeadReckonTaskWithIMU(this, signalPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");

                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
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
        // straife to left align with ground junction
        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,4, 0.5);
        // back to align with wall since straife drifts
        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,2, -0.5);
        // drive up to ground junction
        driveToGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.5);

        // back away from low junction
        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,8, -0.5);
        // strife to the left
        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,9, -0.5);
        // back up to align with the wall
        driveFromGround1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,3, -0.5);


        // lifts to small junction
        liftToSmallJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.7);


        // lowers lift to the Ground Junction
        lowerLiftToGroundJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.6);

        // raise lift off the Ground Junction
        raiseLiftOffGroundJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, 0.65);


        // Based on Signal ID:
        // return to initial to go forward then to the left


        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,18, -0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);


        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
        ;

        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 1.5, -0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,16, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
    }

    @Override
    public void init()
    {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        armServo = hardwareMap.servo.get("armServo");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectIMUDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId","none");
        initPaths();
    }

    @Override
    public void start()
    {
        liftToFirstGroundJunction();
        whereAmI.setValue("in Start");
        setAprilTagDetection();
        addTask(detectionTask);
    }
}

