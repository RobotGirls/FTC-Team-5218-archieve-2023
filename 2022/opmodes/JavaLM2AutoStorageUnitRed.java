/*
Copyright (c) September 2017 FTC Teams 25/5218

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FTC Teams 25/5218 nor the names of their contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.OneWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;


@Autonomous(name = "JavaLM2AutoStorageUnitRed")
//@Disabled
public class JavaLM2AutoStorageUnitRed extends Robot {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor carouselMech;
    private OneWheelDirectDrivetrain singleMotorDrivetrain;
    private OneWheelDirectDrivetrain liftMotorDrivetrain;
    private OneWheelDirectDrivetrain intakeMotorDrivetrain;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;

    private FourWheelDirectDrivetrain drivetrain;

    DeadReckonPath firstPath;
    DeadReckonPath secondPath;
    DeadReckonPath carouselPath;
    DeadReckonPath initialLiftPath;
    DeadReckonPath initialPath;
    DeadReckonPath shippingPath;
    DeadReckonPath liftPath;
    DeadReckonPath intakePath;

    /*
     * The default event handler for the robot.
     */
    @Override
    public void handleEvent(RobotEvent e)
    {
        /*
         * Every time we complete a segment drop a note in the robot log.
         */
        if (e instanceof DeadReckonTask.DeadReckonEvent) {
            RobotLog.i("Completed path segment %d", ((DeadReckonTask.DeadReckonEvent)e).segment_num);
        }
    }

    public void spinCarousel()
    {
        this.addTask(new DeadReckonTask(this, carouselPath, singleMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("spun carousel");
                    parkInStorageUnit();
                }
            }
        });
    }

    public void initialLift()
    {
        this.addTask(new DeadReckonTask(this, initialLiftPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("lifted to second tier");
                    initialJump();
                }
            }
        });
    }
    public void liftToSecondTier()
    {
        this.addTask(new DeadReckonTask(this, liftPath, liftMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("lifted to second tier");
                    depositInTier();
                }
            }
        });
    }

    public void depositInTier()
    {
        this.addTask(new DeadReckonTask(this, intakePath, intakeMotorDrivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("deposited into tier");
                }
            }
        });
    }

    public void initPaths()
    {
        firstPath = new DeadReckonPath();
        secondPath = new DeadReckonPath();
        carouselPath = new DeadReckonPath();
        initialPath = new DeadReckonPath();
        initialLiftPath = new DeadReckonPath();
        shippingPath = new DeadReckonPath();
        liftPath = new DeadReckonPath();
        intakePath = new DeadReckonPath();

        firstPath.stop();
        secondPath.stop();
        carouselPath.stop();
        initialPath.stop();
        initialLiftPath.stop();
        shippingPath.stop();
        liftPath.stop();
        intakePath.stop();

        firstPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2.5, 0.5);
        firstPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 14.5, 0.5);
        firstPath.addSegment(DeadReckonPath.SegmentType.TURN, 9, 0.5);

        secondPath.addSegment(DeadReckonPath.SegmentType.TURN, 9, -0.5);
        secondPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 2, -0.5);
        secondPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 14, 0.5);

        carouselPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, 0.2);

        initialPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 0.3);

        initialLiftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 13, -0.2);

        shippingPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 11, 0.3);
        shippingPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 7.5, 0.3);

        liftPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 25, -0.2);

        intakePath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 5, 1);
    }

    public void goToCarousel()
    {
        this.addTask(new DeadReckonTask(this, firstPath, drivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("went forward to carousel");
                    spinCarousel();
                }
            }
        });
    }

    public void initialJump()
    {
        this.addTask(new DeadReckonTask(this, initialPath, drivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("jumped off wall");
                    goToShippingHub();
                }
            }
        });
    }
    public void goToShippingHub()
    {
        this.addTask(new DeadReckonTask(this, shippingPath, drivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("went to shipping hub");
                    liftToSecondTier();
                }
            }
        });
    }

    public void parkInStorageUnit()
    {
        this.addTask(new DeadReckonTask(this, secondPath, drivetrain){
            @Override
            public void handleEvent (RobotEvent e){
                DeadReckonEvent path = (DeadReckonEvent) e;
                if (path.kind == EventKind.PATH_DONE)
                {
                    RobotLog.i("parked in storage unit");
                }
            }
        });
    }
    @Override
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carouselMech = hardwareMap.get(DcMotor.class, "carouselMech");
        liftMotor = hardwareMap.get(DcMotor.class,"liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        carouselMech.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drivetrain = new FourWheelDirectDrivetrain(frontRight, backRight, frontLeft, backLeft);
        drivetrain.resetEncoders();
        drivetrain.encodersOn();

        singleMotorDrivetrain = new OneWheelDirectDrivetrain(carouselMech);
        singleMotorDrivetrain.resetEncoders();
        singleMotorDrivetrain.encodersOn();

        liftMotorDrivetrain = new OneWheelDirectDrivetrain(liftMotor);
        liftMotorDrivetrain.resetEncoders();
        liftMotorDrivetrain.encodersOn();

        intakeMotorDrivetrain = new OneWheelDirectDrivetrain(intakeMotor);
        intakeMotorDrivetrain.resetEncoders();
        intakeMotorDrivetrain.encodersOn();

        initPaths();
    }

    @Override
    public void start()
    {
        initialLift();
    }
}
