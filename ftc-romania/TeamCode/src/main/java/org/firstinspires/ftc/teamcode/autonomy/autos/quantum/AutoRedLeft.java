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

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(fin)
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(900,0.2,0);
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
                .splineTo(new Vector2d(60,-10),Math.toRadians(-10))
                .build();

        Trajectory park = bot.trajectoryBuilder(takeRings.end())
                .lineTo(new Vector2d(70,0))
                .build();

         /*
        ============================================================== 1 RING ===================================================================
         */
        Trajectory putAwayWobble11 = bot.trajectoryBuilder(fin,true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    moveWobble(900,0.2,200);
                })
                .lineToLinearHeading(new Pose2d(101, -9, Math.toRadians(90)))
                .build();

        Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble11.end())
                .splineTo(new Vector2d(108,18),Math.toRadians(180))
                .splineTo(new Vector2d(80,0), Math.toRadians(180))
                .splineTo(new Vector2d(45,-18), Math.toRadians(180))
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
                .lineToLinearHeading(new Pose2d(63,-10,Math.toRadians(-1)))
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

        shootPowershots();

        if(zona == 0)
        {
            bot.followTrajectory(putAwayWobble10);
            releaseWobble();
            moveWobble(0,0.3,300);
            bot.followTrajectory(goForward);
            bot.followTrajectory(takeRings);
            servoBlock.open();
            sleep(600);
            shoot(3);
            bot.followTrajectory(park);
        }
        else if(zona == 1)
        {
            bot.followTrajectory(putAwayWobble11);
            releaseWobble();
            bot.followTrajectory(returnBack);
            bot.followTrajectory(trajShooting);
            sleep(500); //wait for intake to take the ring
            servoBlock.open();
            sleep(500);
            shoot(3);
            bot.followTrajectory(park1);
        }
    }
}
