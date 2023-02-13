package opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import team25core.Robot;
import team25core.RobotEvent;
import team25core.RunToEncoderValueTask;

@Config
@Autonomous(name = "LiftTest")
public class LiftTest extends Robot {

    public static int REV_40_TO_1_COUNTS_PER_REV = 1120;
    public static int REV_20_TO_1_COUNTS_PER_REV = 560;

    public static int LOWER_LIFT_ENC_COUNTS = 2 * REV_40_TO_1_COUNTS_PER_REV;
    public static double MOTOR_POWER = 0.5;

    DcMotor liftMotor;

    @Override
    public void handleEvent(RobotEvent e) {

    }

    @Override
    public void init() {
        super.init();

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        this.addTask(new RunToEncoderValueTask(this, liftMotor, LOWER_LIFT_ENC_COUNTS, MOTOR_POWER) {
            @Override
            public void handleEvent(RobotEvent e) {
                RunToEncoderValueTask.RunToEncoderValueEvent evt = (RunToEncoderValueEvent) e;
                if (evt.kind == EventKind.DONE) {

                    RobotLog.i("liftedToLowJunction");
                }
            }
        });

    }
}
