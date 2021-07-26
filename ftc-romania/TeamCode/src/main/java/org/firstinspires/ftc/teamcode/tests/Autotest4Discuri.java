package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.util.Arrays;


@Autonomous
public class Autotest4Discuri extends LinearOpMode {

    public SampleMecanumDrive bot;
    public double powershotPower = 1420;
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

        Trajectory goToRings = bot.trajectoryBuilder(trajShoot.end(), true)
                .splineToConstantHeading(new Vector2d(22, -18),0)
                .build();

        Trajectory take1Ring = bot.trajectoryBuilder(goToRings.end())
                .strafeTo(new Vector2d(34,-21),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(23, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0,()->{
                    bot.intakeMotor.setPower(1);
                })
                .build();

        Trajectory getBack = bot.trajectoryBuilder(take1Ring.end())
                .lineToLinearHeading(new Pose2d(30,-20, Math.toRadians(-4)))
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .build();

        Trajectory goShoot = bot.trajectoryBuilder(getBack.end())
                .strafeTo(new Vector2d(53,-20))
                .splineTo(new Vector2d(63.5,-16),Math.toRadians(-3))
                .addTemporalMarker(0,()->{
                    servoBlock.close();
                    bot.intakeMotor.setPower(1);
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .build();

        Trajectory putAwayWobble1 = bot.trajectoryBuilder(goShoot.end())
                .splineToLinearHeading(new Pose2d(115,-30, Math.toRadians(90)),0)
                .addTemporalMarker(0,()->{
                    bot.wobbleMotor.setTargetPosition(950);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        Trajectory park = bot.trajectoryBuilder(putAwayWobble1.end())
                .lineToLinearHeading(new Pose2d(70,-30,Math.toRadians(0)))
                .addTemporalMarker(0, ()->{
                    bot.wobbleMotor.setTargetPosition(0);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        bot.followTrajectory(trajShoot);
        shoot(1, false);
        bot.turn(Math.toRadians(-6));
        shoot(1,false);
        bot.turn(Math.toRadians(-6));
        shoot(1,false);
        bot.outtakeMotor.setVelocity(0);
        bot.followTrajectory(goToRings);
        servoBlock.close();
        bot.followTrajectory(take1Ring);
        bot.followTrajectory(getBack);
        sleep(700);
        bot.intakeMotor.setPower(0);
        servoBlock.setServoPositions(0.68);
        sleep(400);
        shoot(2,false);
        bot.followTrajectory(goShoot);
        sleep(1700);
        bot.intakeMotor.setPower(0);
        servoBlock.open();
        sleep(400);
        shoot(3,false);
        bot.outtakeMotor.setVelocity(0);
        bot.followTrajectory(putAwayWobble1);
        servoWobble.open();
        sleep(200);
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
            if(i!=rings)
                sleep(380);
            else
                sleep(200);

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