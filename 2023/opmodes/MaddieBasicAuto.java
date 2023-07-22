package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.MaddiesDriveTrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;
import team25core.StandardFourMotorRobot;

@Autonomous (name = "MaddieAuto",group = "java")
public class MaddieBasicAuto extends StandardFourMotorRobot {


   private static String TAG = "Maddie";
    private FourWheelDirectDrivetrain drivetrain;

    private OneWheelDirectDrivetrain frontLeftWheelDrivetrain;
    private OneWheelDirectDrivetrain frontRightWheelDrivetrain;
    private OneWheelDirectDrivetrain backLeftWheelDrivetrain;
    private OneWheelDirectDrivetrain backRightWheelDrivetrain;

    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private DcMotor liftMotor;



    private Telemetry.Item eventTlm;
    private DeadReckonPath leftPath;
    private DeadReckonPath oneWheelPath;

    public static int RAISE_LIFT = 1500;






    public void driveMaddiesPath(FourWheelDirectDrivetrain theDrivetrain)
    {
        this.addTask(new DeadReckonTask(this, leftPath, theDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent pathEvent = (DeadReckonEvent) e;
                if (pathEvent.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("left path is done");
                    eventTlm.setValue("path is done");
                    raiseLiftMotorPath();

                }
                else if(pathEvent.kind == EventKind.SEGMENT_DONE){
                    eventTlm.setValue("segment is done");

                }
            }
        });
    }
    public void raiseLiftMotorPath() {
        this.addTask(new RunToEncoderValueTask(this, liftMotor, RAISE_LIFT , 1) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueEvent evt = (RunToEncoderValueEvent)e;
                if (evt.kind == EventKind.DONE) {
                    RobotLog.i("raiseLift");

                }
            }
        });
    }


    public void init() {
        super.init(); //taking parents init(hardware mapping)
        //instantiating the MaddiesDriveTrain
//        frontLeftWheelDrivetrain = new OneWheelDirectDrivetrain(frontLeft);
//        frontLeftWheelDrivetrain.resetEncoders();
//        frontLeftWheelDrivetrain.encodersOn();

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//
//        frontRightWheelDrivetrain = new OneWheelDirectDrivetrain(frontRight);
//        frontRightWheelDrivetrain.resetEncoders();
//        frontRightWheelDrivetrain.encodersOn();
//
//        backLeftWheelDrivetrain = new OneWheelDirectDrivetrain(backLeft);
//        backLeftWheelDrivetrain.resetEncoders();
//        backLeftWheelDrivetrain.encodersOn();
//
//        backRightWheelDrivetrain = new OneWheelDirectDrivetrain(backRight);
//        backRightWheelDrivetrain.resetEncoders();
//        backRightWheelDrivetrain.encodersOn();
        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft); // constructor is instantiating MaddiesDriveTrain class

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();
        // uncomment and call this method only if the robot is going opposite direction from expecteed
        // drivetrain.setCanonicalMotorDirection()

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();
        //Sets the behavior of the motor when a power level of zero is applied ie. stop moving
       // drivetrain.brakeOnZeroPower();

        //sets the motor encoder position to zero
        drivetrain.resetEncoders();

        eventTlm = telemetry.addData("pathEvent", "none");

        //enumeration: variable type similar to init
        initPaths();
    }

    private void initPaths() {
        leftPath = new DeadReckonPath();
        leftPath. stop();
        oneWheelPath = new DeadReckonPath();
        oneWheelPath. stop();

        oneWheelPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10, 0.5);



        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,10, 0.5);
        leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,6, 0.6);
        leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,6, -0.6);



    }
    public void myDrive(FourWheelDirectDrivetrain theDrivetrain)
    {
        this.addTask(new DeadReckonTask(this,oneWheelPath, theDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent pathEvent = (DeadReckonEvent) e;
                if (pathEvent.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("liftedToGroundJunction");
                    eventTlm.setValue("path is done");

                }
                else if(pathEvent.kind == EventKind.SEGMENT_DONE){
                    eventTlm.setValue("segment is done");

                }
            }
        });
    }

    @Override
    public void start(){
        RobotLog.ii(TAG,"start");
        eventTlm.setValue("I'm in start");
        driveMaddiesPath(drivetrain);

    }


}

