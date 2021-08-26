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
                .lineToLinearHeading(new Pose2d(80,29,Math.toRadians(-100)))
                .build();

        Trajectory takeRings = bot.trajectoryBuilder(putAwayWobble10.end())
                .addTemporalMarker(0,()->{
                    bot.intakeMotor.setPower(0.8);
                    servoBlock.close();
                })
                .splineTo(new Vector2d(111,31), Math.toRadians(-90))
                .splineTo(new Vector2d(110,-6), Math.toRadians(-90))
                .splineTo(new Vector2d(96,20),Math.toRadians(90))
                .build();

        Trajectory trajShooting = bot.trajectoryBuilder(takeRings.end(),true)
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .splineTo(powershotVectorBlue,Math.toRadians(15))
                .build();

        Trajectory park = bot.trajectoryBuilder(trajShooting.end())
                .lineToLinearHeading(new Pose2d(70,-10,Math.toRadians(0)))
                .build();

         /*
        ============================================================== 1 RING ===================================================================
         */
        Trajectory putAwayWobble11 = bot.trajectoryBuilder(new Pose2d(powershotVectorBlue,highAngleBlue),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-870,0.2,200);
                    bot.intakeMotor.setPower(0.8);
                    servoBlock.close();
                })
                .splineTo(new Vector2d(100, 40), Math.toRadians(90))
                .build();

        Trajectory takeRings1 = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(111,31), Math.toRadians(-90))
                .splineTo(new Vector2d(110,-6), Math.toRadians(-90))
                .splineTo(new Vector2d(96,20),Math.toRadians(90))
                .build();

        Trajectory trajShooting1 = bot.trajectoryBuilder(takeRings1.end(),true)
                .splineTo(powershotVectorBlue,Math.toRadians(15))
                .build();


        Trajectory park1 = bot.trajectoryBuilder(trajShooting1.end())
                .splineTo(new Vector2d(70,-6), Math.toRadians(-180))
                .build();

        /*
        ============================================================== 4 RINGS ===================================================================
         */

        Trajectory putAwayWobble14 = bot.trajectoryBuilder(new Pose2d(powershotVectorBlue,powershotAngleBlue),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    bot.intakeMotor.setPower(0.8);
                    servoBlock.close();
                    moveWobble(-850,0.2,200);
                })
                .splineTo(new Vector2d(100, 40), Math.toRadians(90))
                .splineTo(new Vector2d(111,31), Math.toRadians(-90))
                .build();

        Trajectory takeRings4 = bot.trajectoryBuilder(putAwayWobble14.end())
                .splineTo(new Vector2d(110,-6), Math.toRadians(-90))
                .splineTo(new Vector2d(96,20),Math.toRadians(90))
                .build();

        Trajectory trajShooting4 = bot.trajectoryBuilder(takeRings4.end(),true)
                .splineTo(powershotVectorBlue,Math.toRadians(15))
                .build();

        Trajectory park4 = bot.trajectoryBuilder(trajShooting4.end())
                .lineToLinearHeading(new Pose2d(70,0,Math.toRadians(0)))
                .build();


        while(!isStarted())
            recognizeRings();

        waitForStart();

        tfod.shutdown();
        moveWobble(-300,0.2,0);

        shootPowershotsBlue();

        if(zona == 0)
        {
            bot.followTrajectory(putAwayWobble10);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(takeRings);
            bot.followTrajectory(trajShooting);
            servoBlock.open();
            bot.intakeMotor.setPower(0);
            sleep(500);
            shoot(3);
            bot.followTrajectory(park);
        }
        else if(zona == 1)
        {
            bot.followTrajectory(putAwayWobble11);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(takeRings1);
            bot.followTrajectory(trajShooting1);
            servoBlock.open();
            bot.intakeMotor.setPower(0);
            sleep(500);
            shoot(3);
            bot.followTrajectory(park1);
        }
        else if(zona == 4)
        {
            bot.followTrajectory(putAwayWobble14);
            releaseWobble();
            moveWobble(0,0.3,0);
            bot.followTrajectory(takeRings4);
            bot.followTrajectory(trajShooting4);
            bot.intakeMotor.setPower(0);
            servoBlock.open();
            sleep(500);
            shoot(3);
            bot.followTrajectory(park4);
        }
    }
}
