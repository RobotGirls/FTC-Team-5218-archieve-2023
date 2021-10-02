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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.FourWheelDirectDrivetrain;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.SingleShotTimerTask;


@Autonomous(name = "JavaLM0AutoBlue")
//@Disabled
public class JavaLM0AutoBlue extends Robot {

    private final static int CAROUSEL_TIMER = 4000;
    private Telemetry.Item loggingTlm;
    private Telemetry.Item objectSeenTlm;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private FourWheelDirectDrivetrain drivetrain;

    private Telemetry.Item timerTlm;
    private Telemetry.Item handleEventTlm;

    SingleShotTimerTask carouselTimerTask;
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

    public void startCarouselTimer() {
        carouselTimerTask = new SingleShotTimerTask(this, CAROUSEL_TIMER) {
            //the handleEvent method is called when timer expires
            @Override
            public void handleEvent(RobotEvent e) {
                SingleShotTimerTask.SingleShotTimerEvent event = (SingleShotTimerEvent) e;

                if (event.kind == EventKind.EXPIRED) {
                    carouselTimerTask.stop();
                    timerTlm.setValue("waited 4 seconds");
                }
            }
        };
    }

    public void parkInStorageUnit()
    {
        DeadReckonPath firstPath = new DeadReckonPath();
        DeadReckonPath secondPath = new DeadReckonPath();
        firstPath.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 30, 0.5);
        secondPath.addSegment(DeadReckonPath.SegmentType.SIDEWAYS, 30, -0.5);
        //path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, 1.0);

        /*
         * Alternatively, this could be an anonymous class declaration that implements
         * handleEvent() for task specific event handlers.
         */
        this.addTask(new DeadReckonTask(this, firstPath, drivetrain));
        startCarouselTimer();
        this.addTask(new DeadReckonTask(this, secondPath, drivetrain));
    }
    @Override
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        drivetrain = new FourWheelDirectDrivetrain(frontRight, rearRight, frontLeft, rearLeft);
    }

    @Override
    public void start()
    {
       parkInStorageUnit();
    }
}
