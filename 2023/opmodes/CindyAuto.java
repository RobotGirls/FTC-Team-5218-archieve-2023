package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;

import team25core.CindysMecanumFourWheelDrivetrain;
import team25core.RobotEvent;
import team25core.RobotTask;
import team25core.StandardFourMotorRobot;


@Autonomous(name="Cindy's auto",group="java-rnrr")

public class CindyAuto extends StandardFourMotorRobot {

    CindysMecanumFourWheelDrivetrain drivetrain;


    @Override
    public void init() {
        super.init();

    }

}
