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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;
import team25core.vision.apriltags.AprilTagDetectionTask;


@Autonomous(name = "javabotsLM1AutoRight1")
//@Disabled
//@Config
public class javabotsLM1AutoRight extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

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

    DeadReckonPath driveToMedium1Path;
    DeadReckonPath driveFromMedium1Path;
    DeadReckonPath liftToMediumJunctionPath;
    DeadReckonPath lowerLiftFirstMediumJunctionPath;
    DeadReckonPath raiseLiftOffFirstMediumJunctionPath;

    DeadReckonPath driveUpToMediumPath;


    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;


    private Telemetry.Item whereAmI;
    private Telemetry.Item myLocation;
    private Telemetry.Item myTag;



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
        myLocation.setValue("one");
        detectionTask = new AprilTagDetectionTask(this, "Webcam 1") {
            @Override
            public void handleEvent(RobotEvent e) {
                myLocation.setValue("two");
                TagDetectionEvent event = (TagDetectionEvent) e;
                tagObject = event.tagObject;
                tagIdTlm.setValue(tagObject.id);
                whereAmI.setValue("in handleEvent");
                detectedAprilTagID = tagObject.id;
                myTag.setValue(detectedAprilTagID);


            }
        };
        whereAmI.setValue("setAprilTagDetection");
        detectionTask.init(telemetry, hardwareMap);
        myLocation.setValue("three");

    }


    public void liftToFirstMediumJunction()
    {
        this.addTask(new DeadReckonTask(this, liftToMediumJunctionPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToMediumJunction");
                    //delay(1);
                    driveUpMediumJunction(driveUpToMediumPath);
                }
            }
        });
    }
    public void lowerLiftFirstMediumJunction()
    {
        whereAmI.setValue(" in lowerLiftFirstMediumJunction ");
        this.addTask(new DeadReckonTask(this, lowerLiftFirstMediumJunctionPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("lower liftToMediumJunction");
                    //delay(1);
                    coneServo.setPosition(CONE_RELEASE);
                    raiseLiftOffFirstMediumJunction();
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



    private void driveToMediumJunction(DeadReckonPath driveToMedium1Path)
    {
        whereAmI.setValue("in drivetoMediumJunction");
        RobotLog.i("drives to First medium Junction");

        this.addTask(new DeadReckonTask(this, driveToMedium1Path, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if(path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    liftToFirstMediumJunction();


                }
            }
        });
    }



    private void driveUpMediumJunction(DeadReckonPath driveToMedium1Path)
    {
        whereAmI.setValue("in drive Up To Medium Junction");
        RobotLog.i("drives to First medium Junction");

        this.addTask(new DeadReckonTask(this, driveUpToMediumPath, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if(path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");
                    lowerLiftFirstMediumJunction();


                }
            }
        });
    }

    public void raiseLiftOffFirstMediumJunction()
    {
        this.addTask(new DeadReckonTask(this, raiseLiftOffFirstMediumJunctionPath, liftMotorDrivetrain){

            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToGroundJunction");

                    driveFromMediumJunction(driveFromMedium1Path);

                }
            }
        });
    }

    private void driveFromMediumJunction(DeadReckonPath driveFromMedium1Path)
    {
        whereAmI.setValue("in driveFromFirstMediumJunction");
        RobotLog.i("drives to First medium Junction");

        this.addTask(new DeadReckonTask(this, driveFromMedium1Path, drivetrain){
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if(path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("finished parking");

                    decideWhichSignalWasSeen();



                }
            }
        });
    }


    public void driveToSignalZone(DeadReckonPath signalPath)
    {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");

        this.addTask(new DeadReckonTask(this, signalPath, drivetrain){
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
                //lowerLiftToFirstGroundJunction();
            }
        });
    }

    public void initPaths()
    {
        driveFromMedium1Path = new DeadReckonPath();
        liftToMediumJunctionPath = new DeadReckonPath();
        lowerLiftFirstMediumJunctionPath = new DeadReckonPath();
        raiseLiftOffFirstMediumJunctionPath = new DeadReckonPath();
        driveToMedium1Path = new DeadReckonPath();
        driveUpToMediumPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath = new DeadReckonPath();

        driveToMedium1Path.stop();
        driveUpToMediumPath.stop();
        driveFromMedium1Path.stop();
        liftToMediumJunctionPath.stop();
        lowerLiftFirstMediumJunctionPath.stop();
        raiseLiftOffFirstMediumJunctionPath.stop();

        leftPath.stop();
        middlePath.stop();
        rightPath.stop();


        // drive straight to beside medium junction
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,20, 0.6);
        // turn to medium junction
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.TURN,27.3, -0.6);
        // drive towards medium junction

        // raise life to medium junction
        liftToMediumJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 35, 0.7); //50

        driveUpToMediumPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,2.5, .7);

        // lowers lift to the medium Junction
        lowerLiftFirstMediumJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, -0.6);

        // raise lift off the medium Junction
        raiseLiftOffFirstMediumJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 6, 0.65);

        // back away from medium junction
        driveFromMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT,2, -0.5);



        // Based on Signal ID:
        // return to initial to go forward then to the left

        // strife to left
        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3, -0.5);
        // go back to align with the wall
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 14, 0.5);
        // go straignt
       // leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);

        // return to initial then go forward
        middlePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        ;
        // return to initial then go forward then right
        // straife to right
        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3, -0.5);
        // go back align with wall
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 14, -0.5);
        // go straight
       // rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 0.5);
    }

    @Override
    public void init()
    {
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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId","none");
        myLocation = telemetry.addData("my location", "none");
        myTag = telemetry.addData("my tag","none");


        initPaths();
    }

    @Override
    public void start()
    {
        whereAmI.setValue("in Start");
        setAprilTagDetection();
        addTask(detectionTask);
        driveToMediumJunction(driveToMedium1Path);

    }
}
