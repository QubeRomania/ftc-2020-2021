package org.firstinspires.ftc.teamcode.autonomy.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomy.AutoBase;

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
