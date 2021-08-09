package org.firstinspires.ftc.teamcode.autonomy.autos.quantum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomy.AutoBase;

@Autonomous
public class AutoRedRight extends AutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        cameraPosition("redright");

        /*
        ============================================================== 0 RINGS ===================================================================
         */

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(fin)
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(900,0.1,0);
                })
                .lineToLinearHeading(new Pose2d(65,-3,Math.toRadians(180)))
                .build();

        Trajectory waitForAlliance = bot.trajectoryBuilder(putAwayWobble10.end())
                .lineToLinearHeading(new Pose2d(28,-3,Math.toRadians(180)))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory park = bot.trajectoryBuilder(waitForAlliance.end(), true)
                .splineTo(new Vector2d(70,10),Math.toRadians(0))
                .build();

         /*
        ============================================================== 1 RING ===================================================================
         */
        Trajectory putAwayWobble11 = bot.trajectoryBuilder(fin,true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(900,0.2,200);
                })
                .lineToLinearHeading(new Pose2d(84, 10, Math.toRadians(-140)))
                .build();

        Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(50,2), Math.toRadians(110))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .addTemporalMarker(0.3,()->{
                    moveWobble(0,0.2,0);
                })
                .build();

        Trajectory takeRing = bot.trajectoryBuilder(returnBack.end())
                .strafeTo(new Vector2d(45,12))
                .build();

        Trajectory trajShooting = bot.trajectoryBuilder(takeRing.end())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .lineToLinearHeading(new Pose2d(highVectorRed,highAngleRed))
                .build();

        Trajectory park1 = bot.trajectoryBuilder(trajShooting.end())
                .lineTo(new Vector2d(70,0))
                .build();

        /*
        ============================================================== 4 RINGS ===================================================================
         */



        while(!isStarted())
            recognizeRings();

        waitForStart();

        tfod.shutdown();

        shoothighGoalRed();

        if(zona == 0)
        {
            bot.followTrajectory(putAwayWobble10);
            releaseWobble();
            bot.followTrajectory(waitForAlliance);
            moveWobble(0,0.3,1000);
            bot.followTrajectory(park);
        }
        else if(zona == 1)
        {
            bot.followTrajectory(putAwayWobble11);
            releaseWobble();
            bot.followTrajectory(returnBack);
            bot.followTrajectory(takeRing);
            bot.intakeMotor.setPower(0);
            servoBlock.open();
            sleep(500);
            bot.followTrajectory(trajShooting);
            sleep(500); //wait for intake to take the ring
            servoBlock.open();
            sleep(500); //wait the block to get up
            shoot(3);
            bot.followTrajectory(park1);
        }
        else if(zona == 4)
        {

        }
    }
}
