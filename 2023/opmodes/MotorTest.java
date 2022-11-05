package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.Drivetrain;
import team25core.Robot;
import team25core.RobotEvent;


@Autonomous(name = "MotorTest")
//@Disabled
public class MotorTest extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private DeadReckonPath FLPath;
    private DeadReckonPath FRPath;
    private DeadReckonPath BRPath;
    private DeadReckonPath BLPath;

    @Override
    public void handleEvent(RobotEvent e) {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent) e).segment_num);
        }
    }

    @Override
    public void init()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }






}

