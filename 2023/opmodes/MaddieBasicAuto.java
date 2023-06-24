package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.MaddiesDriveTrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.RobotEvent;
import team25core.StandardFourMotorRobot;

@Autonomous (name = "MaddieAuto",group = "java")
public class MaddieBasicAuto extends StandardFourMotorRobot {

    private MaddiesDriveTrain drivetrain;

    private OneWheelDirectDrivetrain frontLeftWheel;
    private OneWheelDirectDrivetrain frontRightWheel;
    private OneWheelDirectDrivetrain backLeftWheel;
    private OneWheelDirectDrivetrain backRightWheel;


    private Telemetry.Item eventTlm;
    private DeadReckonPath leftPath;
    private DeadReckonPath oneWheelPath;

    public void oneWheelDrive(OneWheelDirectDrivetrain theWheel)
    {
        this.addTask(new DeadReckonTask(this,oneWheelPath, theWheel){
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

    public void driveMaddiesPath()
    {
        this.addTask(new DeadReckonTask(this, leftPath, drivetrain){
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
        //driveMaddiesPath();
        oneWheelDrive(frontLeftWheel);
        oneWheelDrive(frontRightWheel);
        oneWheelDrive(backLeftWheel);
        oneWheelDrive(backRightWheel);




    }
    public void init() {
        super.init(); //taking parents init(hardware mapping)
        //instantiating the MaddiesDriveTrain
        frontLeftWheel = new OneWheelDirectDrivetrain(frontLeft);
        frontLeftWheel.resetEncoders();
        frontLeftWheel.encodersOn();

        frontRightWheel = new OneWheelDirectDrivetrain(frontRight);
        frontRightWheel.resetEncoders();
        frontRightWheel.encodersOn();

        backLeftWheel = new OneWheelDirectDrivetrain(backLeft);
        backLeftWheel.resetEncoders();
        backLeftWheel.encodersOn();

        backRightWheel = new OneWheelDirectDrivetrain(backRight);
        backRightWheel.resetEncoders();
        backRightWheel.encodersOn();
        drivetrain = new MaddiesDriveTrain(frontRight, backRight, frontLeft, backLeft); // contrustor is instantiating MaddiesDriveTrain class

        // uncomment and call this method only if the robot is going opposite direction from expecteed
        // drivetrain.setCanonicalMotorDirection()

        //motor will try to tun at the targeted velocity
        drivetrain.encodersOn();
        //Sets the behavior of the motor when a power level of zero is applied ie. stop moving
        drivetrain.brakeOnZeroPower();

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
        //leftPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS,6, 0.6);
       // leftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT,6, -0.6);



    }
}

