package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import team25core.MaddiesDriveTrain;
import team25core.StandardFourMotorRobot;

@Autonomous (name = "MaddieAuto",group = "java")
public class MaddieBasicAuto extends StandardFourMotorRobot {

    private MaddiesDriveTrain drivetrain;

    @Override
    public void init() {
        super.init(); //taking parents init(hardware mapping)


        drivetrain = new MaddiesDriveTrain(frontRight, backRight, frontLeft, backLeft); // instantiating MaddiesDriveTrain class

        // uncomment and call this method only if the robot is going opposite direction from expecteed
        // drivetrain.setCanonicalMotorDirection()
        drivetrain.encodersOn();

        drivetrain.breakOnZeroPower();

        //sets the motor encoder position to zero
        drivetrain.resetEncoders();

        //enumeration: variable type similar to int





    }

}

