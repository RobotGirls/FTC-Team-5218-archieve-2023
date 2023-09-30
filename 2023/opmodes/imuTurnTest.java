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
import team25core.DeadReckonTaskWithIMUTurn;
import team25core.FourWheelDirectIMUDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;
import team25core.sensors.color.RGBColorSensorTask;
import team25core.vision.apriltags.AprilTagDetectionTask;

@Autonomous(name = "imuTurnTest")
@Config
public class imuTurnTest extends Robot {

    //declarations
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private FourWheelDirectIMUDrivetrain drivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;
    private boolean goingToMediumJunction = true;

    private ColorSensor groundColorSensor;

    //Servos
    private Servo coneServo;
    private Servo armServo;

    public static final double CONE_GRAB = 0.12;
    public static final double CONE_RELEASE = 1.00;

    private static final double ARM_FRONT = .875;
    private static final double ARM_BACK = 0.0918;


    //All Paths

    //Drive To Mid Junction
    DeadReckonPath driveToMedium1Path;
    DeadReckonPath driveFromMedium1Path;
    DeadReckonPath liftToMediumJunctionPath;

    // Signal Zone Parking
    private DeadReckonPath leftPath;
    private DeadReckonPath middlePath;
    private DeadReckonPath rightPath;

    public static int SIGNAL_LEFT = 5;
    public static int SIGNAL_MIDDLE = 2;
    public static int SIGNAL_RIGHT = 18;
    public static double FORWARD_DISTANCE = 6;
    public static double DRIVE_SPEED = -0.5;

    //IMU
    private BNO055IMU imu;
    private DeadReckonTaskWithIMUTurn gyroTask;
    private Telemetry.Item headingTlm;
    private static final double TARGET_YAW_FOR_DRIVING_STRAIGHT = 90.0;
    private boolean showHeading = true;
    private boolean usingSmoothStart = false;
    private boolean isUsingImuTurns = true;
    private Telemetry.Item targetYawTlm;
    Telemetry.Item currentYawTlm;
    Telemetry.Item yawErrorTlm;

    public static double TURN_AMOUNT = 75;


    Telemetry.Item imuStatusTlm;
    Telemetry.Item imuCalibTlm;
    Telemetry.Item imuHeadingTlm;
    Telemetry.Item imuYawRateTlm;
    Telemetry.Item imuRollTlm;
    Telemetry.Item imuPitchTlm;
    Telemetry.Item imuGravTlm;

    Telemetry.Item segmentTypeTlm;
    Telemetry.Item codeLocation;
    Telemetry.Item deltaHeadingTlm;

    Telemetry.Item hitTargetHeadingTlm;
    Telemetry.Item hitHeadingTlm;

    //ColorSensor
    protected RGBColorSensorTask colorSensorTask;

    //AprilTags
    AprilTagDetection tagObject;
    private AprilTagDetectionTask detectionTask;
    private Telemetry.Item tagIdTlm;
    private int detectedAprilTagID;

    //telemetry
    private Telemetry.Item whereAmI;

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent) e).segment_num);
        }
    }

    public void initImuTelemetry() {

        imuStatusTlm = telemetry.addData("Status", "none");
        imuCalibTlm = telemetry.addData("Calib","none");
        imuHeadingTlm = telemetry.addData("Heading", "none");
        imuYawRateTlm = telemetry.addData("YawRate", "none");

        imuRollTlm = telemetry.addData("Roll", "none");
        imuPitchTlm = telemetry.addData("Pitch", "none");
        imuGravTlm = telemetry.addData("Grav", "none");
        segmentTypeTlm = telemetry.addData("segmentType", "none");
        telemetry.setMsTransmissionInterval(100);
        // FIXME potentially restore transmission interval

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

    public void liftToFirstMediumJunction() {
        whereAmI.setValue("in liftToFirstMediumJunction");

        this.addTask(new RunToEncoderValueTask(this, liftMotor, 1900 , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("liftedToLowJunction");
                  driveToFromFirstMediumJunction(driveToMedium1Path);
                }
            }
        });
    }

    private void driveToFromFirstMediumJunction(DeadReckonPath driveToMedium1Path) {
        whereAmI.setValue("in driveToFromFirstMediumJunction");
        RobotLog.i("drives to First Low Junction");
        gyroTask = new DeadReckonTaskWithIMUTurn(this, driveToMedium1Path, drivetrain, usingSmoothStart, isUsingImuTurns) {
            @Override
            public void handleEvent(RobotEvent e) {
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE) {
                    RobotLog.i("finished parking");
                    if (goingToMediumJunction) {
                       // lowerLiftToFirstMediumJunction();
                        goingToMediumJunction = false;
                        whereAmI.setValue("in driveToFromFirstMediumJunction in if");

                    } else {
                     //   raiseLiftOffFirstMediumJunction();
                        whereAmI.setValue("in driveToFromFirstMediumJunction in else");

                    }
                }
            }
        };
        gyroTask.initializeImu(imu, (double) TARGET_YAW_FOR_DRIVING_STRAIGHT, showHeading, headingTlm);
        gyroTask.initTelemetry(imuCalibTlm, imuGravTlm, imuRollTlm, imuPitchTlm, imuHeadingTlm,
                imuStatusTlm, imuYawRateTlm, whereAmI, segmentTypeTlm, codeLocation, deltaHeadingTlm,
                hitTargetHeadingTlm,hitHeadingTlm);
        addTask(gyroTask);
    }

        public void lowerLiftToFirstMediumJunction() {
            whereAmI.setValue("in lowerLiftToFirstMediumJunction");
            this.addTask(new RunToEncoderValueTask(this, liftMotor, 1000 , -1) {
                @Override
                public void handleEvent(RobotEvent e) {
                    RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                    if (evt.kind == EventKind.DONE) {
                        RobotLog.i("liftedToLowJunction");
                        coneServo.setPosition(CONE_RELEASE);
                      //raiseLiftOffFirstLowJunction();

                        //armServo.setPosition(ARM_FRONT);
                    }
                }
            });
        }

        public void raiseLiftOffFirstMediumJunction() {
            whereAmI.setValue("in raiseLiftOffFirstMediumJunction ");

            this.addTask(new RunToEncoderValueTask(this, liftMotor, 300 , 1) {
                @Override
                public void handleEvent(RobotEvent e) {
                    RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                    if (evt.kind == EventKind.DONE) {
                        RobotLog.i("liftedToLowJunction");
                      //  driveToFromFirstLowJunction(driveFromLow1Path);
                        armServo.setPosition(ARM_FRONT);
                    }
                }
            });
        }

    public void initPaths() {
        driveToMedium1Path = new DeadReckonPath();
        driveFromMedium1Path = new DeadReckonPath();
        liftToMediumJunctionPath = new DeadReckonPath();

        leftPath = new DeadReckonPath();
        middlePath = new DeadReckonPath();
        rightPath = new DeadReckonPath();

        driveToMedium1Path.stop();
        driveFromMedium1Path.stop();
        liftToMediumJunctionPath.stop();

        leftPath.stop();
        middlePath.stop();
        rightPath.stop();

        //Drive to medium junction
      // driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, 0.55);
        //need to turn based on heading

      //  driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 3, 0.55);
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.TURN_WITH_IMU, TURN_AMOUNT, 0.55);
      //  driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, -0.55);
        driveToMedium1Path.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 10  , 0.55);

        //drive from medium
       // driveFromMedium1Path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, -0.65);


        // Based on Signal ID:
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 0.70, -1);
        middlePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10, -1);
        rightPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 24, -0.6);
    }

    @Override
    public void init() {
        super.init();

        //drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetYawTlm = telemetry.addData("target yaw", "none");
        currentYawTlm = telemetry.addData("current yaw", "none");
        yawErrorTlm = telemetry.addData("yaw error", "none");

        drivetrain = new FourWheelDirectIMUDrivetrain(frontRight, backRight,
                frontLeft, backLeft, targetYawTlm, yawErrorTlm, currentYawTlm);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();

        //lift
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        //servos
        //FIXME commenting out servos
//        coneServo = hardwareMap.servo.get("coneServo");
//        armServo = hardwareMap.servo.get("armServo");
//        coneServo.setPosition(CONE_GRAB);
//        armServo.setPosition(ARM_FRONT);

        //colorsensor
        //FIXME add colorsensor back in
       // groundColorSensor = hardwareMap.colorSensor.get("groundColorSensor");

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // We are setting the target yaw to the value of the constant
        // TARGET_YAW_FOR_DRIVING_STRAIGHT which is zero for our robot to
        // go straight.
        drivetrain.setTargetYaw(TARGET_YAW_FOR_DRIVING_STRAIGHT);
        initImuTelemetry();

        //Telemetry
        headingTlm = telemetry.addData("Current/target heading is: ", "0.0");
        codeLocation = telemetry.addData("Code Location is ", "none");
        deltaHeadingTlm = telemetry.addData("Delta Heading", "none");
//        colorDetectedTlm = telemetry.addData("color detected", "unknown");
//
//        blueDetectedTlm = telemetry.addData("blue color sensor value", 0);
//        redDetectedTlm = telemetry.addData("red color sensor value", 0);
//        greenDetectedTlm = telemetry.addData("green color sensor value", 0);

        hitTargetHeadingTlm = telemetry.addData("Hit Target Heading", "none");
        hitHeadingTlm = telemetry.addData("Hit Heading", "none");
        whereAmI = telemetry.addData("location in code", "init");
        tagIdTlm = telemetry.addData("tagId", "none");
        initPaths();

    }


    @Override
    public void start() {

        driveToFromFirstMediumJunction(driveToMedium1Path);
        //liftToFirstMediumJunction();
        whereAmI.setValue("in Start");
       // setAprilTagDetection();
      //  addTask(detectionTask);
    }

}
