package test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import team25core.AutopilotTask;
import team25core.BNO055Gyro;
import team25core.MRGyro;
import team25core.MechanumGearedDrivetrain;
import team25core.RobotEvent;
import team25core.RobotGyro;
import team25core.SingleShotTimerTask;
import team25core.StandardFourMotorRobot;

@Autonomous(name="Autopilot Strafe Test")
//@Disabled
public class AutopilotStrafeTest extends StandardFourMotorRobot {

    BNO055IMU imu;
    ModernRoboticsI2cGyro gyro;
    RobotGyro robotGyro;

    private final static double K_P = 0.03;
    private final static double K_I = 0.0001;
    private final static double K_D = 0.02;

    private MechanumGearedDrivetrain drivetrain;

    @Override
    public void handleEvent(RobotEvent e) { super.handleEvent(e); }

    public void initializeMRGyro()
    {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();
        robotGyro = new MRGyro(gyro);
    }

    public void initializeRevIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robotGyro = new BNO055Gyro(imu);
    }

    @Override
    public void init()
    {
        super.init();

        gyro = null;
        imu = null;

        drivetrain = new MechanumGearedDrivetrain(motorMap);
        drivetrain.setMasterMotor(frontRight);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();
        drivetrain.setNoncanonicalMotorDirection();

        initializeMRGyro();
    }

    public void pause()
    {
        this.addTask(new SingleShotTimerTask(this, 2000) {
            @Override
            public void handleEvent(RobotEvent e) {
                doLeftStrafe();
            }
        });
    }

    public void doLeftStrafe()
    {
        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }

        drivetrain.strafe(-0.5);
        final AutopilotTask autopilot = new AutopilotTask(this, robotGyro, AutopilotTask.TrackingAxis.AXIS_Z, AutopilotTask.AutoPilotDirection.RIGHT, motorMap);
        autopilot.setKvalues(K_P, K_I, K_D);
        this.addTask(autopilot);

        this.addTask(new SingleShotTimerTask(this, 2700) {
            @Override
            public void handleEvent(RobotEvent e) {
                removeTask(autopilot);
                drivetrain.stop();
                pause();
            }
        });
    }

    public void doNoAutopilot()
    {
        drivetrain.strafe(0.5);
        this.addTask(new SingleShotTimerTask(this, 2700) {
            @Override
            public void handleEvent(RobotEvent e) {
                drivetrain.stop();
            }
        });
    }

    @Override
    public void start()
    {
        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }

        drivetrain.strafe(0.5);
        final AutopilotTask autopilot = new AutopilotTask(this, robotGyro, AutopilotTask.TrackingAxis.AXIS_Z, AutopilotTask.AutoPilotDirection.RIGHT, motorMap);
        autopilot.setKvalues(K_P, K_I, K_D);
        this.addTask(autopilot);

        this.addTask(new SingleShotTimerTask(this, 2700) {
            @Override
            public void handleEvent(RobotEvent e) {
                removeTask(autopilot);
                drivetrain.stop();
                pause();
            }
        });
    }
}