package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive.PoseStorage;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;


@Autonomous
public class Autotest1Disc extends LinearOpMode {

    public SampleMecanumDrive bot;
    public double powershotPower = 1410;
    public double towerPower = 1620;
    public double blocPos = 0.17;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();
    servo_perete servoPerete = new servo_perete();

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(52, 0, 4.6, 14);

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        bot.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(powershotPower);
                })
                .splineTo(new Vector2d(63.5, 7), Math.toRadians(-1))
                .build();

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(trajShoot.end(),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToLinearHeading(new Pose2d(103, -9, Math.toRadians(90)))
                .build();

        Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble10.end())
                .splineTo(new Vector2d(108,15),Math.toRadians(180))
                .splineTo(new Vector2d(80,0), Math.toRadians(180))
                .splineTo(new Vector2d(45,-20), Math.toRadians(180))
                .addTemporalMarker(1,()->{
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory goBack = bot.trajectoryBuilder(returnBack.end())
                .splineToLinearHeading(new Pose2d(34,-18,Math.toRadians(30)),0)
                .build();

        Trajectory takeWobble2 = bot.trajectoryBuilder(goBack.end())
                .lineTo(new Vector2d(27,-25))
                .build();

        Trajectory trajShooting = bot.trajectoryBuilder(takeWobble2.end())
                .addTemporalMarker(0, ()->{
                    servoBlock.open();
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .lineToLinearHeading(new Pose2d(63,-20,Math.toRadians(-2)))
                .build();

        Trajectory putAwayWobble2 = bot.trajectoryBuilder(trajShooting.end())
                .lineToLinearHeading(new Pose2d(79,-25, Math.toRadians(180)))
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(0);
                })
                .build();

        Trajectory park = bot.trajectoryBuilder(putAwayWobble2.end())
                .lineTo(new Vector2d(70,-25))
                .build();

        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        bot.followTrajectory(trajShoot);
        shoot(1, false);
        bot.turn(Math.toRadians(-6.1));
        shoot(1,false);
        bot.turn(Math.toRadians(-6));
        shoot(1,false);
        bot.outtakeMotor.setVelocity(0);
        bot.followTrajectory(putAwayWobble10);
        servoWobble.open();
        sleep(200);
        bot.wobbleMotor.setTargetPosition(300);
        bot.wobbleMotor.setPower(0.2);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(300);
        bot.followTrajectory(returnBack);
        bot.followTrajectory(goBack);
        bot.followTrajectory(takeWobble2);
        servoWobble.close();
        sleep(300);
        bot.followTrajectory(trajShooting);
        bot.intakeMotor.setPower(0);
        sleep(200);
        shoot(3,false);
        bot.followTrajectory(putAwayWobble2);
        servoWobble.open();
        sleep(100);
        bot.wobbleMotor.setTargetPosition(0);
        bot.wobbleMotor.setPower(0.2);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(300);
        bot.followTrajectory(park);

    }

    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            bot.pistonMotor.setPower(0.3);
            bot.pistonMotor.setTargetPosition(-170);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(380);
            bot.pistonMotor.setTargetPosition(0);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(400);

            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(200);
            }
        }
    }
}