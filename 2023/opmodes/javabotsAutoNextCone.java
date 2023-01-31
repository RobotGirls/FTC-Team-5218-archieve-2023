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
import team25core.FourWheelDirectIMUDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
import team25core.vision.apriltags.AprilTagDetectionTask;


@Autonomous(name = "javabotsAutoNextCone2")
//@Disabled
//@Config
public class javabotsAutoNextCone extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectIMUDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private boolean goingToLowerJunction = true;

    private boolean goingToHighJunction = true;

    private Servo coneServo;
    private Servo armServo;

    private static final double CONE_GRAB = 0.12;
    private static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = .875;
    private static final double ARM_BACK = 0.0918;
    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    static final int SIGNAL_LEFT = 5;
    static final int SIGNAL_MIDDLE = 2;
    static final int SIGNAL_RIGHT = 18;
    static final double FORWARD_DISTANCE = 6;
    static final double DRIVE_SPEED = -0.5;

    DeadReckonPath driveToLow1Path;
    DeadReckonPath driveFromLow1Path;
    DeadReckonPath liftToLowJunctionPath;
    DeadReckonPath lowerLiftToLowJunctionPath;
    DeadReckonPath raiseLiftOffLowJunctionPath;
    DeadReckonPath lowerLiftToHighJunctionPath;

    DeadReckonPath coneStackPath;
    DeadReckonPath lowerLiftToConeStackPath;
    DeadReckonPath raiseLiftOffConeStackPath;
    DeadReckonPath driveToHighJunctionPath;
    DeadReckonPath driveFromHighJunctionPath;
    DeadReckonPath raiseLiftToHighJunctionPath;

    private BNO055IMU imu;
    private DeadReckonTaskWithIMU gyroTask;
    private Telemetry.Item headingTlm;

    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;

    private Telemetry.Item whereAmI;

    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 0.0;
    private boolean showHeading = true;

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

    public void liftToFirstLowJunction()
    {
        this.addTask(new DeadReckonTask(this, liftToLowJunctionPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLoewJunction");
                 driveToFromFirstLowJunction(driveToLow1Path);
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


    private void driveToFromFirstLowJunction(DeadReckonPath driveToLow1Path)
    {
        whereAmI.setValue("in driveToFromFirstLowJunction");
        RobotLog.i("drives to First Low Junction");
        gyroTask = new DeadReckonTaskWithIMU(this, driveToLow1Path, drivetrain)
        //this.addTask(new DeadReckonTask(this, driveToLow1Path, drivetrain)
        {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    if (goingToLowerJunction) {
                        lowerLiftToFirstLowJunction();
                        goingToLowerJunction = false;
                    } else {
                        // have finished backing away from lower junction
                        // going to go Cone Stack
                       driveToConeStack(coneStackPath);
                    }
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
    }


    public void lowerLiftToFirstLowJunction()
    {
        this.addTask(new DeadReckonTask(this, lowerLiftToLowJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");

                    coneServo.setPosition(CONE_RELEASE);
                    raiseLiftOffFirstLowJunction();
                }
            }
        });
    }

    public void raiseLiftOffFirstLowJunction()
    {
        this.addTask(new DeadReckonTask(this, raiseLiftOffLowJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");
                    //driveToConeStack(coneStackPath);
                    driveToFromFirstLowJunction(driveFromLow1Path);

                }
            }
        });
    }

    public void driveToConeStack(DeadReckonPath coneStackPath)
    {
        whereAmI.setValue("driveToConeStack");
        RobotLog.i("drives to First Low Junction");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStackPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    // lowers lift for the first time to the cone stack
                    lowerLiftToConeStackFirst();
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
    }



    public void lowerLiftToConeStackFirst()
    {

        this.addTask(new DeadReckonTask(this, lowerLiftToConeStackPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");

                    coneServo.setPosition(CONE_GRAB);
                    raiseLiftOffConeStackFirst();
                }
            }
        });
    }
    
    public void raiseLiftOffConeStackFirst()
    {

        this.addTask(new DeadReckonTask(this, raiseLiftOffConeStackPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");

                    coneServo.setPosition(CONE_GRAB);

                   // driveToFromFirstHighJunction(driveToHighJunctionPath);
                }
            }
        });
    }

    public void driveToFromFirstHighJunction(DeadReckonPath driveToHighJunctionPath)
    {
        whereAmI.setValue("in driveToFromFirstLowJunction");
        RobotLog.i("drives to First Low Junction");

        this.addTask(new DeadReckonTaskWithIMU(this, driveToHighJunctionPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    if (goingToHighJunction) {
                        raiseLiftToHighJunction();
                        goingToHighJunction = false;
                    } else {
                        // cone has been put on high junction
                        decideWhichSignalWasSeen();
                    }
                }
            }
        });
    }

    public void raiseLiftToHighJunction()
    {

        this.addTask(new DeadReckonTask(this, raiseLiftToHighJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");

                    lowerLiftToHighJunction();

                }
            }
        });
    }

    public void  lowerLiftToHighJunction()
    {

        this.addTask(new DeadReckonTask(this, lowerLiftToHighJunctionPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    driveToFromFirstHighJunction(driveFromHighJunctionPath);
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

//    public void delayAndLowerLift(int milliseconds)
//    {
//        addTask(new SingleShotTimerTask(this, 1000*milliseconds) {
//            @Override
//            public void handleEvent (RobotEvent e) {
//                        lowerLiftToFirstLowJunction();
//            }
//        });
//    }

    public void initPaths()
    {
        driveToLow1Path = new DeadReckonPath();
        driveFromLow1Path = new DeadReckonPath();
        liftToLowJunctionPath = new DeadReckonPath();
        lowerLiftToLowJunctionPath = new DeadReckonPath();
        raiseLiftOffLowJunctionPath = new DeadReckonPath();

        coneStackPath = new DeadReckonPath();
        lowerLiftToConeStackPath = new DeadReckonPath();
        raiseLiftOffConeStackPath = new DeadReckonPath();
        driveToHighJunctionPath = new DeadReckonPath();
        driveFromHighJunctionPath = new DeadReckonPath();
        raiseLiftToHighJunctionPath = new DeadReckonPath();
        lowerLiftToHighJunctionPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath= new DeadReckonPath();

        driveToLow1Path.stop();
        driveFromLow1Path.stop();
        liftToLowJunctionPath.stop();
        lowerLiftToLowJunctionPath.stop();
        raiseLiftOffLowJunctionPath.stop();


        lowerLiftToConeStackPath.stop();
        raiseLiftOffConeStackPath.stop();
        driveToHighJunctionPath.stop();
        raiseLiftToHighJunctionPath.stop();
        lowerLiftToHighJunctionPath.stop();

        leftPath.stop();
        middlePath.stop();
        rightPath.stop();
        coneStackPath.stop();

//        // drives to first low junction
//        // strafe to left align with low junction
        driveToLow1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,8, 0.45);
////        // drive up to ground junction
        driveToLow1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.55);
//
        // strife to the left
        liftToLowJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.9);

//        driveFromLow1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,9, -0.5);

        lowerLiftToLowJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, -0.9);

        raiseLiftOffLowJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, 0.8);
//        // back away from low junction
        driveFromLow1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,2, -0.5); //neg
//

        // random path ...

       coneStackPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 8.4,  -0.50);
       // coneStackPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 25,  -0.50);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 22.5,  0.8);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.TURN, 25.5,  -0.50);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13,  0.50);
      //  coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20,  0.65); // neg
//
//        // lower lift to cone stack
        lowerLiftToConeStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 9.5, -0.6); //distance or 8
//        // picked up cone lifting lift off of the stack
         raiseLiftOffConeStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.6);
        raiseLiftOffConeStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.6);

        //drives to high junction
       //  driveToHighJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.65);
//        // lower lift to High Junction
//        lowerLiftToHighJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.65);
//        ///drives From HighJunction
//        driveFromHighJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.65);
//
//        // Based on Signal ID:
//        // return to initial to go forward then to the left
//
//         // strife to left
//        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,18, -0.5);
//        // go back to align with the wall
//        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
//        // go straight
//        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
//
//        // return to initial then go forward
//        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.5);
//        ;
//        // return to initial then go forward then right
//        // strafe to right
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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectIMUDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();
        // We are setting the target yaw to the value of the constant
        // TARGET_YAW_FOR_DRIVING_STRAIGHT which is zero for our robot to
        // go straight.
        drivetrain.setTarget(TARGET_YAW_FOR_DRIVING_STRAIGHT);

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        // initialized heading telemetry
        headingTlm = telemetry.addData("Current/target heading is: ", "0.0");

        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId","none");
        initPaths();


    }

    @Override
    public void start()
    {
        liftToFirstLowJunction();
        whereAmI.setValue("in Start");
        setAprilTagDetection();
        addTask(detectionTask);
    }
}

