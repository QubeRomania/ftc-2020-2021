package org.firstinspires.ftc.teamcode.autonomy.autos.quantum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomy.AutoBase;

@Autonomous
public class AutoRedLeft extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        cameraPosition("redleft");

        /*
        ============================================================== 0 RINGS ===================================================================
         */

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(new Pose2d(powershotVectorRed,powershotAngleRed))
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-900,0.2,0);
                })
                .lineToLinearHeading(new Pose2d(80,-25,Math.toRadians(90)))
                .build();

        Trajectory goForward = bot.trajectoryBuilder(putAwayWobble10.end())
                .splineTo(new Vector2d(118,-34), Math.toRadians(90))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory takeRings = bot.trajectoryBuilder(goForward.end())
                .addTemporalMarker(2,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .strafeTo(new Vector2d(118,5))
                .splineTo(new Vector2d(60,-6),Math.toRadians(-19))
                .build();

        Trajectory park = bot.trajectoryBuilder(takeRings.end())
                .lineTo(new Vector2d(70,0))
                .build();

         /*
        ============================================================== 1 RING ===================================================================
         */
        Trajectory putAwayWobble11 = bot.trajectoryBuilder(new Pose2d(powershotVectorRed,powershotAngleRed),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-900,0.2,200);
                })
                .lineToLinearHeading(new Pose2d(101, -9, Math.toRadians(90)))
                .build();

        Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(108,18),Math.toRadians(180))
                .splineTo(new Vector2d(60,-6), Math.toRadians(-19))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .addTemporalMarker(0.5,()->{
                    moveWobble(0,0.2,0);
                })
                .build();


        Trajectory park1 = bot.trajectoryBuilder(returnBack.end())
                .lineTo(new Vector2d(70,0))
                .build();

        /*
        ============================================================== 4 RINGS ===================================================================
         */
        Trajectory putAwayWobble14 = bot.trajectoryBuilder(new Pose2d(powershotVectorRed,powershotAngleRed))
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(-900,0.2,0);
                    servoBlock.close();
                })
                .strafeTo(new Vector2d(80,-2))
                .splineTo(new Vector2d(116,-22),Math.toRadians(90))
                .build();


        Trajectory takeRings4 = bot.trajectoryBuilder(putAwayWobble14.end())
                .addTemporalMarker(0,()->{
                    bot.intakeMotor.setPower(0.8);
                })
                .addTemporalMarker(2,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .strafeTo(new Vector2d(118,5))
                .splineTo(new Vector2d(60,-2),Math.toRadians(-20))
                .build();

        Trajectory park4 = bot.trajectoryBuilder(takeRings4.end())
                .lineToLinearHeading(new Pose2d(70,3,0))
                .build();

        while(!isStarted())
            recognizeRings();

        waitForStart();

        tfod.shutdown();
        moveWobble(-300,0.2,0);

        shootPowershotsRed();

        if(zona == 0)
        {
            bot.followTrajectory(putAwayWobble10);
            releaseWobble();
            bot.followTrajectory(goForward);
            bot.followTrajectory(takeRings);
            servoBlock.open();
            sleep(600);
            shoot(3);
            moveWobble(0,0.3,400);
            bot.followTrajectory(park);
        }
        else if(zona == 1)
        {
            bot.followTrajectory(putAwayWobble11);
            releaseWobble();
            bot.followTrajectory(returnBack);
            sleep(500); //wait for intake to take the ring
            servoBlock.open();
            sleep(500); //wait the block to get up
            shoot(3);
            moveWobble(0,0.3,400);
            bot.followTrajectory(park1);
        }
        else if(zona == 4)
        {
            sleep(1500);
            bot.followTrajectory(putAwayWobble14);
            releaseWobble();
            bot.followTrajectory(takeRings4);
            servoBlock.open();
            sleep(500);
            shoot(3);
            moveWobble(0,0.3,400);
            bot.followTrajectory(park4);
        }
    }
}
