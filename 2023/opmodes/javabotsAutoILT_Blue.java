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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
import team25core.RunToEncoderValueTask;
import team25core.sensors.color.RGBColorSensorTask;
import team25core.vision.apriltags.AprilTagDetectionTask;

@Config
@Autonomous(name = "javabotsAutoILT9_Blue")
//@Disabled
public class javabotsAutoILT_Blue extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectIMUDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private boolean goingToLowerJunction = true;

    private boolean goingToHighJunction = true;

    private ColorSensor groundColorSensor;
    private Servo coneServo;
    private Servo armServo;

    public static final double CONE_GRAB = 0.12;
    public static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = .875;
    private static final double ARM_BACK = 0.0918;
    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    public static int SIGNAL_LEFT = 5;
    public static int SIGNAL_MIDDLE = 2;
    public static int SIGNAL_RIGHT = 18;
    public static double FORWARD_DISTANCE = 6;
    public static double DRIVE_SPEED = -0.5;

    public static int REV_40_TO_1_COUNTS_PER_REV = 1120;
    public static int REV_20_TO_1_COUNTS_PER_REV = 560;

    //public static int LOWER_LIFT_ENC_COUNTS = 2 * REV_40_TO_1_COUNTS_PER_REV;
    public static int LOWER_LIFT_ENC_COUNTS = REV_20_TO_1_COUNTS_PER_REV;

    public static double  CONE_STACK_SIDEWAYS_1 = 7;
    public static double CONE_STACK_STRAIGHT_FORWARD_1 = 10;
    public static double CONE_STACK_STRAIGHT_BACK_1 = 2.5;
    public static double CONE_STACK_STRAIGHT_FORWARD_2 = 29;
    public static double CONE_STACK_STRAIGHT_BACK_2 = 2.0;
    public static double CONE_STACK_STRAIGHT_FWD_SPEED_2 = 0.8;

    public static double TARGET_YAW_FOR_NEW_DIRECTION = -90;


    DeadReckonPath driveToLow1Path;
    DeadReckonPath driveFromLow1Path;
    DeadReckonPath liftToLowJunctionPath;
    DeadReckonPath lowerLiftToLowJunctionPath;
    DeadReckonPath raiseLiftOffLowJunctionPath;
    DeadReckonPath lowerLiftToHighJunctionPath;

    DeadReckonPath coneStackPath;
    DeadReckonPath raiseLiftOffConeStackPath1;
    DeadReckonPath raiseLiftOffConeStackPath2;
    boolean firstConeLift = true;
    DeadReckonPath driveToHighJunctionPath;
    DeadReckonPath driveFromHighJunctionPath;
    DeadReckonPath raiseLiftToHighJunctionPath;

    DeadReckonPath lowerLiftBeforeConeStackPath;
    DeadReckonPath coneStackCloserPath;

    DeadReckonPath coneStackToJunctionPath;

    DeadReckonPath colorDetectionStrafePath;


    private BNO055IMU imu;
    private DeadReckonTaskWithIMU gyroTask;
    private Telemetry.Item headingTlm;

    private Telemetry.Item tagIdTlm;

    private int detectedAprilTagID;

    protected RGBColorSensorTask colorSensorTask;

    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    //private double tagID;
    private Telemetry.Item colorDetectedTlm;
    private Telemetry.Item redDetectedTlm;
    private Telemetry.Item greenDetectedTlm;
    private Telemetry.Item blueDetectedTlm;
    private Telemetry.Item whereAmI;

    // MultipleTelemetry telemetry;

    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 0.0;
    private boolean showHeading = true;

    DeadReckonTask deadReckonTask;

    /*
     * The default event handler for the robot.
     */

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent) e).segment_num);
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
              //  detectionTask.stop();
            }
        };
        whereAmI.setValue("setAprilTagDetection");
        detectionTask.init(telemetry, hardwareMap);
    }

    public void liftToFirstLowJunction() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 1900 , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    // MADDIEFIXME in this current code we're grabbing the cone and raising the
                    // lift at the same time. There is a possibility that we may be raising the lift
                    // before the claw can fully extend. In that case we may have to put a delay
                    // before the lift so we may need to delay.

                    driveToFromFirstLowJunction(driveToLow1Path);


                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }

    private void decideWhichSignalWasSeen() {
        if (detectedAprilTagID == SIGNAL_LEFT) {
            driveToSignalZone(leftPath);
        } else if (detectedAprilTagID == SIGNAL_MIDDLE) {
            driveToSignalZone(middlePath);
        } else {
            driveToSignalZone(rightPath);
        }
    }


    private void driveToFromFirstLowJunction(DeadReckonPath driveToLow1Path) {
        whereAmI.setValue("in driveToFromFirstLowJunction");
        RobotLog.i("drives to First Low Junction");
        gyroTask = new DeadReckonTaskWithIMU(this, driveToLow1Path, drivetrain)
                //this.addTask(new DeadReckonTask(this, driveToLow1Path, drivetrain)
        {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                    if (goingToLowerJunction) {
                        lowerLiftToFirstLowJunction();
                        goingToLowerJunction = false;
                    } else {
                        // have finished backing away from lower junction
                        // going to go Cone Stack
//                        raiseLiftOffFirstLowJunction();
                        lowerLiftBeforeConeStack();
                    }
                }
            }
        };
//        if (goingToLowerJunction) {
            gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
            gyroTask.initTelemetry(this.telemetry);
            addTask(gyroTask);
//        }
    }



    public void lowerLiftToFirstLowJunction() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 1000 , -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    raiseLiftOffFirstLowJunction();

                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }

    public void raiseLiftOffFirstLowJunction() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 300 , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    driveToFromFirstLowJunction(driveFromLow1Path);
                    armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }


    public void lowerLiftBeforeConeStack() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 1 , -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    // MADDIEFIXME in this current code we're grabbing the cone and raising the
                    // lift at the same time. There is a possibility that we may be raising the lift
                    // before the claw can fully extend. In that case we may have to put a delay
                    // before the lift so we may need to delay.
                    driveToConeStack(coneStackPath);

                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }
   

    public void driveToConeStack(DeadReckonPath coneStackPath) {
        whereAmI.setValue("driveToConeStack");
        RobotLog.i("drives to First Low Junction");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStackPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when path done we are approximatly a foot away from the cone stack
                if (path.kind == EventKind.PATH_DONE) {
                    // as we're driving closer to the cone stack we're simultaneously lifting the lift
                    // drivetrain.setTargetYaw(TARGET_YAW_FOR_NEW_DIRECTION);
                  //  Stack(raiseLiftOffConeStackPath1);raiseLiftOffCone
                    colorDetectionStrafe();
                    armServo.setPosition(ARM_FRONT);
                   // driveCloserToConeStack(coneStackCloserPath);

                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }


    public void driveCloserToConeStack(DeadReckonPath coneStackCloserPath) {
        whereAmI.setValue("driveToConeStack");
        RobotLog.i("drives to First Low Junction");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStackCloserPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    lowerLiftToGrabConeOnStack();
                    coneServo.setPosition(CONE_GRAB);
                  //  raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                  //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }


    public void lowerLiftToGrabConeOnStack() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 600, -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                    // MADDIEFIXME in this current code we're grabbing the cone and raising the
                    // lift at the same time. There is a possibility that we may be raising the lift
                    // before the claw can fully extend. In that case we may have to put a delay
                    // before the lift so we may need to delay.

                    raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                   // decideWhichSignalWasSeen();
                    //armServo.setPosition(ARM_FRONT);
                }
            }
        });
    }



    public void colorDetectionStrafe() {
        whereAmI.setValue("colorDetectionStrafe");
        handleColorSensor();
        gyroTask = new DeadReckonTaskWithIMU(this, colorDetectionStrafePath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    // FIXME add driveCloserToConeStack
                    armServo.setPosition(ARM_FRONT);
                  //  raiseLiftOffConeStack(raiseLiftOffConeStackPath1);
                 //   driveCloserToConeStack(coneStackCloserPath);
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        // FIXME Add the initTlm back in
//        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }

    public void handleColorSensor () {
        whereAmI.setValue("handleColorSensor");
        colorSensorTask = new RGBColorSensorTask(this, groundColorSensor) {
            public void handleEvent(RobotEvent e) {
                whereAmI.setValue("handleColorSensor handleEvent");
                ColorSensorEvent event = (ColorSensorEvent) e;
                colorArray = colorSensorTask.getColors();
                blueDetectedTlm.setValue(colorArray[0]);
                redDetectedTlm.setValue(colorArray[1]);
                greenDetectedTlm.setValue(colorArray[2]);
                switch(event.kind) {
                    // red is at the end
                    case RED_DETECTED:
                        drivetrain.stop();
                        robot.removeTask(colorSensorTask);
                        gyroTask.resume();
                      // raiseLiftOffConeStack(raiseLiftOffConeStackPath1);
                       driveCloserToConeStack(coneStackCloserPath);
                        colorDetectedTlm.setValue("red");
                        break;
                    case BLUE_DETECTED:
                        drivetrain.stop();
                        gyroTask.resume();
                        colorDetectedTlm.setValue("blue");
                        break;
                    default:
                        colorDetectedTlm.setValue("none");
                        break;
                }
            }
        };
        colorSensorTask.setThresholds(10000, 10000, 5000);
        colorSensorTask.setDrivetrain(drivetrain);
        addTask(colorSensorTask);
    }

    public void raiseLiftOffConeStack(DeadReckonPath coneStackLiftPath) {
        whereAmI.setValue("in raiseLiftOffConeStack");
        // we're raising the lift so it doesn't crash into the cone stack
        this.addTask(new DeadReckonTask(this, coneStackLiftPath, liftMotorDrivetrain) {

            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when this path is done we just raised the lift so it doesn't collide with the cone stack
                if (path.kind == EventKind.PATH_DONE) {
                    if (firstConeLift) {
                      //  driveCloserToConeStack(coneStackCloserPath);
                        // now lower the lift in order to grab the cone
                       //lowerLiftToGrabConeOnStack();
                       //coneServo.setPosition(CONE_GRAB);
                       // driveCloserToConeStack(coneStackCloserPath);
                       // driveCloserToConeStack();
                         decideWhichSignalWasSeen();
                        firstConeLift = false;
                    } else {
                        // at this point we have already lifted the cone off the cone stack
                        // MADDIE FIXME (we either lift the cone only high enough to clear the
                        // cone stack or high enough to clear the low junction we can change
                        // raiseLiftOffConeStackPath2)

                        // now we'll move the arm to the back
                       // armServo.setPosition(ARM_BACK);
                       // decideWhichSignalWasSeen();
                       // driveFromConeStackToJunction();
                    }


                }
            }
        });
    }

    public void lowerLiftToConeStackPath() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_ENC_COUNTS, -1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("loweredLiftTo2ndLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    whereAmI.setValue("finished lowerLiftToConeStackPath");


                }
            }
        });
    }


    public void driveFromConeStackToJunction() {
        whereAmI.setValue("driveFromConeStackToJunction");

        gyroTask = new DeadReckonTaskWithIMU(this, coneStackToJunctionPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                // when the path is done we have completed the driveCloserToConeStack
                if (path.kind == EventKind.PATH_DONE) {
                    raiseLiftOffConeStack(raiseLiftOffConeStackPath2);
                  //  lowerLiftTo2ndLowJunctionPath();
                    //do nothing here
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        //gyroTask.useSmoothStart(true);
        addTask(gyroTask);
    }

    public void lowerLiftTo2ndLowJunctionPath() {
        whereAmI.setValue("in lowerLiftToGrabConeOnStack");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_ENC_COUNTS, -0.20) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("loweredLiftTo2ndLowJunction");
                    coneServo.setPosition(CONE_RELEASE);
                    whereAmI.setValue("finished lowerLiftToConeStackPath");
                  //  decideWhichSignalWasSeen();

                }
            }
        });
    }

    public void driveToSignalZone(DeadReckonPath signalPath) {
        whereAmI.setValue("in driveToSignalZone");
        RobotLog.i("drives straight onto the launch line");
        gyroTask = new DeadReckonTaskWithIMU(this, signalPath, drivetrain) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(this.telemetry);
        addTask(gyroTask);
    }

    public void initPaths() {
        driveToLow1Path = new DeadReckonPath();
        driveFromLow1Path = new DeadReckonPath();
        liftToLowJunctionPath = new DeadReckonPath();
        lowerLiftToLowJunctionPath = new DeadReckonPath();
        raiseLiftOffLowJunctionPath = new DeadReckonPath();

        coneStackPath = new DeadReckonPath();
        raiseLiftOffConeStackPath1 = new DeadReckonPath();
        raiseLiftOffConeStackPath2 = new DeadReckonPath();
        driveToHighJunctionPath = new DeadReckonPath();
        driveFromHighJunctionPath = new DeadReckonPath();
        raiseLiftToHighJunctionPath = new DeadReckonPath();
        lowerLiftToHighJunctionPath = new DeadReckonPath();

        lowerLiftBeforeConeStackPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath = new DeadReckonPath();
        coneStackCloserPath = new DeadReckonPath();
        coneStackToJunctionPath = new DeadReckonPath();

        colorDetectionStrafePath = new DeadReckonPath();

        driveToLow1Path.stop();
        driveFromLow1Path.stop();
        liftToLowJunctionPath.stop();
        lowerLiftToLowJunctionPath.stop();
        raiseLiftOffLowJunctionPath.stop();

        raiseLiftOffConeStackPath1.stop();
        raiseLiftOffConeStackPath2.stop();
        driveToHighJunctionPath.stop();
        raiseLiftToHighJunctionPath.stop();
        lowerLiftToHighJunctionPath.stop();

        lowerLiftBeforeConeStackPath.stop();
        coneStackCloserPath.stop();
        coneStackToJunctionPath.stop();

        colorDetectionStrafePath.stop();


        leftPath.stop();
        middlePath.stop();
        rightPath.stop();
        coneStackPath.stop();

//        // drives to first low junction
//        // strafe to left align with low junction
        driveToLow1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 5.4, 0.50);
////        // drive up to ground junction
        driveToLow1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.55);

        driveFromLow1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.65);


//        // back away from low junction
       // driveFromLow1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.6); //neg
//
       // coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5.0, -0.55);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, CONE_STACK_SIDEWAYS_1, -0.45); //6.3
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, CONE_STACK_STRAIGHT_BACK_1, -0.45); //2 ,-.5
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, CONE_STACK_STRAIGHT_FORWARD_2, CONE_STACK_STRAIGHT_FWD_SPEED_2);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, CONE_STACK_STRAIGHT_BACK_2, -0.9);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.TURN, 27.3, -0.40);
        coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 7, 0.5);
        //coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, 0.5); //going to conestack
        //  coneStackPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20,  0.65); // neg


        colorDetectionStrafePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 4, 0.4);
        // strafes to the tape
        // MADDIEFIXME adjust the drive closer path as necessary so it doesn't ram in to the cone stack
        coneStackCloserPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.25); //distance 7
//
        // MADDIEFIXME raise the lift before the cone stack so it doesn't collide with it may need to ADJUST how high to raise the lift
        raiseLiftOffConeStackPath1.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, 1.0);

        // MADDIEFIXME raise the lift in order to lift the cone above the cone stack
        raiseLiftOffConeStackPath2.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 20, 1.0);

        // MADDIEFIXME drive back to the lower junction may have to adjust the distance
        coneStackToJunctionPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 2, -0.5);
        //coneStackToJunctionPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 3.2, -0.1);

        // Based on Signal ID:
        // return to initial to go forward then to the left

         // Stays in first parking spot
      //  leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3.2, - 0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.70, -1);

        // return to initial then go forward
       // middlePath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3.2, 0.5);
        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10, -1);
        ;
        // return to initial then go forward then right
        // strafe to right
       // rightPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,3.2, 0.5);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 24, -0.6);
     //   rightPath.addSegment(DeadReckonPath.SegmentType.TURN, 2, -0.75);


        //30
    }

    @Override
    public void init() {
        // cindy added
        super.init();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        coneServo = hardwareMap.servo.get("coneServo");
        armServo = hardwareMap.servo.get("armServo");
        groundColorSensor = hardwareMap.colorSensor.get("groundColorSensor");

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
        drivetrain.setTargetYaw(TARGET_YAW_FOR_DRIVING_STRAIGHT);

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        coneServo.setPosition(CONE_GRAB);
        armServo.setPosition(ARM_FRONT);

        // initialized heading telemetry
        headingTlm = telemetry.addData("Current/target heading is: ", "0.0");

        colorDetectedTlm = telemetry.addData("color detected", "unknown");

        blueDetectedTlm = telemetry.addData("blue color sensor value", 0);
        redDetectedTlm = telemetry.addData("red color sensor value", 0);
        greenDetectedTlm = telemetry.addData("green color sensor value", 0);

        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        initPaths();

    }

    @Override
    public void start() {

        liftToFirstLowJunction();
        whereAmI.setValue("in Start");
        setAprilTagDetection();
        addTask(detectionTask);

        // liftToFirstLowJunction();
        // FIXME Start from Color detection strafe to test 2nd junction
       // colorDetectionStrafe();
    }
}

