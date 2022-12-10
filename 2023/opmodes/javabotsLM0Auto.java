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

import team25core.Drivetrain;
import team25core.vision.apriltags.AprilTagDetectionPipeline;
import team25core.vision.apriltags.AprilTagDetectionTask;
import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.MechanumGearedDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;


@Autonomous(name = "javabotsLM0Auto2")
//@Disabled
public class javabotsLM0Auto extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;

    private Servo coneServo;
    private Servo armServo;

    private static final double CONE_GRAB = 0.35;
    private static final double CONE_RELEASE = 0.6;

    private static final double ARM_FRONT = 0.06;
    private static final double ARM_BACK = 1;
    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    //private DeadReckonPath depositInFirstGroundJunctionPath;

    //private DeadReckonPath initialConeGrabPath;

    static final int SIGNAL_LEFT = 5;
    static final int SIGNAL_MIDDLE = 2;
    static final int SIGNAL_RIGHT = 18;
    static final double FORWARD_DISTANCE = 6;
    static final double DRIVE_SPEED = -0.5;

    DeadReckonPath ground1Path;
    DeadReckonPath liftToSmallJunctionPath;
    DeadReckonPath secondPath;

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
                    delay(1);
//                 driveToFirstGroundJunction(ground1Path);
                }
            }
        });
    }

//    private void driveToFirstGroundJunction(DeadReckonPath ground1Path)
//    {
//        whereAmI.setValue("in driveToFirstGroundJunction");
//        RobotLog.i("drives to First Ground Junction");
//
//        this.addTask(new DeadReckonTask(this, ground1Path, drivetrain){
//            @Override
//            public void handleEvent(RobotEvent e) {
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE)
//                {
//                    RobotLog.i("finished parking");
//                    coneServo.setPosition(CONE_RELEASE);
//
//
//                }
//            }
//        });
//    }

//    public void dropToFirstGroundJunction()
//    {
//        this.addTask(new DeadReckonTask(this, liftToSmallJunctionPath, liftMotorDrivetrain){
//            @Override
//            public void handleEvent (RobotEvent e){
//                DeadReckonEvent path = (DeadReckonEvent) e;
//                if (path.kind == EventKind.PATH_DONE)
//                {
//                    RobotLog.i("liftedToGroundJunction");
//                    driveToFirstGroundJunction(ground1Path);
//                }
//            }
//        });
//    }

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

    public void delay(int seconds)
    {
        this.addTask(new SingleShotTimerTask(this, 1000*seconds) {
            @Override
            public void handleEvent (RobotEvent e){
                SingleShotTimerEvent event = (SingleShotTimerEvent) e;
                switch(event.kind) {
                    case EXPIRED:
                        if (detectedAprilTagID == SIGNAL_LEFT){
                            driveToSignalZone(leftPath);
                        } else if (detectedAprilTagID == SIGNAL_MIDDLE){
                            driveToSignalZone(middlePath);
                        } else {
                            driveToSignalZone(rightPath);
                        }
                        break;
                }
            }
        });
    }



    public void initPaths()
    {
//        secondPath = new DeadReckonPath();
        ground1Path = new DeadReckonPath();
        liftToSmallJunctionPath = new DeadReckonPath();

        ground1Path.stop();
        liftToSmallJunctionPath.stop();
//        secondPath.stop();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath= new DeadReckonPath();

        leftPath.stop();
        middlePath.stop();
        rightPath.stop();

        //drives to first ground junction
//        ground1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,9 , -0.5);
//        ground1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);

        liftToSmallJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 8, 0.5);

        //return to intial to go forward then to the left

//        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
//        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,8.5, 0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 17, -0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);
        //return to intial then go forward
//        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
//        middlePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,8.5, 0.5);
        middlePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 3, 0.5);
        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 17 , 0.5);
        //return to initial then go forward then right
//        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, -0.5);
//        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,8.5, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 20, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 12, 0.5);

        //lift path
        // servo path
        //secondPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, 0.3);
        //driving towards cone stack
        //lift path
        // servo path
        //thirdPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, -0.3);
        //drive back to cone stack
        //lift path
        // servo path
        //fourthPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 15, 0.3);
        //in position to park in the correct position for the fifth path
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

