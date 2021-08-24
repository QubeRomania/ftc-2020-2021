package org.firstinspires.ftc.teamcode.autonomy.autos.quantum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomy.AutoBase;

@Autonomous
public class AutoBlueRight extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        cameraPosition("blueright");


        /*
        ============================================================== 0 RINGS ===================================================================
         */

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(new Pose2d(powershotVectorBlue,powershotAngleBlue))
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-870,0.2,0);
                })
                .lineToLinearHeading(new Pose2d(75,25,Math.toRadians(-100)))
                .build();

        Trajectory park = bot.trajectoryBuilder(putAwayWobble10.end())
                .lineToLinearHeading(new Pose2d(70,-5,Math.toRadians(0)))
                .build();

         /*
        ============================================================== 1 RING ===================================================================
         */
        Trajectory putAwayWobble11 = bot.trajectoryBuilder(new Pose2d(powershotVectorBlue,highAngleBlue),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-870,0.2,200);
                })
                .lineToLinearHeading(new Pose2d(101, -6, Math.toRadians(-90)))
                .build();

        /*Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(80,0), Math.toRadians(180))
                .splineTo(new Vector2d(45,18), Math.toRadians(180))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .addTemporalMarker(0.5,()->{
                    moveWobble(0,0.2,0);
                })
                .build();

        Trajectory trajShooting = bot.trajectoryBuilder(returnBack.end())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .lineToLinearHeading(new Pose2d(63,10,Math.toRadians(-6)))
                .build();
         */

        Trajectory park1 = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(70,-6), Math.toRadians(-180))
                .build();

        /*
        ============================================================== 4 RINGS ===================================================================
         */

        Trajectory putAwayWobble14 = bot.trajectoryBuilder(new Pose2d(powershotVectorBlue,powershotAngleBlue),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-870,0.2,200);
                })
                .lineToLinearHeading(new Pose2d(120,22,Math.toRadians(-90)))
                .build();

        Trajectory park4 = bot.trajectoryBuilder(putAwayWobble14.end())
                .lineToLinearHeading(new Pose2d(70,0,Math.toRadians(0)))
                .build();



        while(!isStarted())
            recognizeRings();

        waitForStart();

        tfod.shutdown();
        moveWobble(-300,0.2,0);

        shootPowershotsBlue();
        sleep(15000);

        if(zona == 0)
        {
            bot.followTrajectory(putAwayWobble10);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(park);
        }
        else if(zona == 1)
        {
            bot.followTrajectory(putAwayWobble11);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(park1);
        }
        else if(zona == 4)
        {
            bot.followTrajectory(putAwayWobble14);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(park4);
        }
    }
}
