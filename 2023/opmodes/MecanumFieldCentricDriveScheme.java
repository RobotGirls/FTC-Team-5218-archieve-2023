package opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.JoystickDriveControlScheme;
import team25core.MotorValues;
import team25core.TeleopDriveTask;

/**
 * Created by Ruchi Bondre on 9/3/22.
 * Editing for Javabots 9/17/22
 */


public class MecanumFieldCentricDriveScheme implements JoystickDriveControlScheme {

    /*
     * An Andymark 40 native spin direction is counterclockwise.
     */

    public enum DriveType {
        DRIVE_GEARED,
        DRIVE_DIRECT,
    };

    public enum MotorPosition {
        OUTER_OPPOSED,
        INNER_OPPOSED,
    };

    public enum MotorDirection {
        CANONICAL,
        NONCANONICAL,
    };


    protected double rx;
    protected double x;
    protected double y;

    protected BNO055IMU imu;

    protected Gamepad gamepad;
    protected MotorDirection motorDirection;
    private Telemetry telemetry;
    private Telemetry.Item fLpowerTlm;
    private Telemetry.Item bLpowerTlm;
    private Telemetry.Item fRpowerTlm;
    private Telemetry.Item bRpowerTlm;

    private Telemetry.Item imuStatusTlm;
    private Telemetry.Item imuCalibTlm;
    private Telemetry.Item imuHeadingTlm;
    private Telemetry.Item imuRollTlm;
    private Telemetry.Item imuPitchTlm;
    private Telemetry.Item imuGravTlm;


    private Telemetry.Item xTlm;
    private Telemetry.Item yTlm;
    private Telemetry.Item cosBotHeadingTlm;
    private Telemetry.Item sinBotHeadingTlm;
    private Telemetry.Item rotXTlm;
    private Telemetry.Item rotYTlm;



    public MecanumFieldCentricDriveScheme(Gamepad gamepad, BNO055IMU imu,  Telemetry telemetry  )
    {
        this.gamepad = gamepad;
        this.motorDirection = MotorDirection.CANONICAL;
        this.imu = imu;

        fLpowerTlm = telemetry.addData("FL Power","unknown");
        fRpowerTlm = telemetry.addData("FR Power","unknown");
        bLpowerTlm = telemetry.addData("BL Power","unknown");
        bRpowerTlm = telemetry.addData("BR Power","unknown");

        xTlm = telemetry.addData("x","unknown");
        yTlm = telemetry.addData("y","unknown");
        cosBotHeadingTlm = telemetry.addData("cosBotHeading","unknown");
        sinBotHeadingTlm = telemetry.addData("sinBotHeading","unknown");
        rotXTlm = telemetry.addData("rotXTlm","unknown");
        rotYTlm = telemetry.addData("rotYTlm","unknown");

//        imuHeadingTlm = telemetry.addData("Heading","unknown");
//        imuRollTlm = telemetry.addData("Pitch","unknown");
//        imuPitchTlm = telemetry.addData("Roll","unknown");
        this.telemetry = telemetry;

    }


    public MecanumFieldCentricDriveScheme(Gamepad gamepad, MotorDirection motorDirection, BNO055IMU imu)
    {
        this.gamepad = gamepad;
        this.motorDirection = motorDirection;
        this.imu = imu;
        this.telemetry = telemetry;
    }


    public void setCanonical(MotorDirection direction)
    {
        this.motorDirection = direction;

    }

    public void initTelemetry(Telemetry.Item imuStatus, Telemetry.Item imuCalib,
                              Telemetry.Item imuHeading,Telemetry.Item imuRoll,
                              Telemetry.Item imuPitch, Telemetry.Item imuGrav)
    {
        this.imuStatusTlm = imuStatus;
        this.imuCalibTlm = imuCalib;
        this.imuHeadingTlm = imuHeading;
        this.imuRollTlm = imuRoll;
        this.imuPitchTlm = imuPitch;
        this.imuGravTlm = imuGrav;
    }

    public void printIMUInfo(BNO055IMU imu)
    {
        // note this would be the heading if the rev hub was mounted
        // horizontally. Since it is not, we have to figure out what
        // this really is.

        double heading = -imu.getAngularOrientation().firstAngle;
        this.imuHeadingTlm.setValue(heading);
        double roll = -imu.getAngularOrientation().secondAngle;
        this.imuRollTlm.setValue(roll);
        double pitch= -imu.getAngularOrientation().thirdAngle;
        this.imuPitchTlm.setValue(pitch);
    }

    public MotorValues getMotorPowers()
    {
        if (this.motorDirection == MotorDirection.CANONICAL ) {
            y = -gamepad.left_stick_y; // Remember, this is reversed!
            rx = gamepad.right_stick_x;
        } else {
            y = gamepad.left_stick_y; // Remember, this is reversed!
            rx = - gamepad.right_stick_x;
        }
        x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing


        // If joysticks are pointed left (negative joystick values), counter rotate wheels.
        // Threshold for joystick values in the x may vary.

        // Assuming the Control Hub or the Rev Hub is horizontal:
        // When we initialize the imu parameters we used AxesOrder.ZYX
        // So we think that Z is the first angle associated with heading
        // then Y is probably the second angles and X is probably the
        // third angle. So we think one of those is pitch and the other
        // is roll
        // However, Javabots Rev Hub is mounted Vertically, so we need
        // to determine which of these axes represents heading

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        printIMUInfo(imu);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        fLpowerTlm.setValue(frontLeftPower);
        fRpowerTlm.setValue(frontRightPower);
        bLpowerTlm.setValue(backLeftPower);
        bRpowerTlm.setValue(backRightPower);

        xTlm.setValue(x);
        yTlm.setValue(y);
        cosBotHeadingTlm.setValue(Math.cos(botHeading));
        sinBotHeadingTlm.setValue(Math.sin(botHeading));
        rotYTlm.setValue(rotY);
        rotXTlm.setValue(rotX);

        return new MotorValues(frontLeftPower, frontRightPower, backLeftPower, backRightPower);






    }
}

