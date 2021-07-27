package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTestPowershots extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        while(!isStarted())
            recognizeRings();

        waitForStart();

        tfod.shutdown();

        shootPowershots();
    }
}
