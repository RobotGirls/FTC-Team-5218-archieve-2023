package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import team25core.MaddiesDriveTrain;
import team25core.StandardFourMotorRobot;

@Autonomous (name = "MaddieAuto",group = "java")
public class MaddieBasicAuto extends StandardFourMotorRobot {

    @Override
    public void init() {
        super.init(); //taking parents init(hardware mapping)

        MaddiesDriveTrain drivetrain;

    }



}