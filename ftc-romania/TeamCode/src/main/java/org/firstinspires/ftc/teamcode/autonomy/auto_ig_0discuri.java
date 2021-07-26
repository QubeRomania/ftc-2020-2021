package org.firstinspires.ftc.teamcode.autonomy;

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


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class auto_ig_0discuri extends LinearOpMode {

    public SampleMecanumDrive bot;
    public double powershotPower = 1450;
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
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .splineTo(new Vector2d(63.5, -17), Math.toRadians(-4))
                .build();

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(trajShoot.end(),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToLinearHeading(new Pose2d(82, -35, Math.toRadians(80)))
                .build();

        Trajectory returnBack = bot.trajectoryBuilder(putAwayWobble10.end())
                .splineToLinearHeading(new Pose2d(34,-18,Math.toRadians(30)),0)
                .addTemporalMarker(1,()->{
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        Trajectory takeWobble2 = bot.trajectoryBuilder(returnBack.end())
                .lineTo(new Vector2d(27,-25))
                .build();

        Trajectory putAwayWobble2 = bot.trajectoryBuilder(takeWobble2.end())
                .lineToLinearHeading(new Pose2d(80,-28, Math.toRadians(60)))
                .build();

        Trajectory park = bot.trajectoryBuilder(putAwayWobble2.end())
                .lineTo(new Vector2d(70,0))
                .build();
        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        bot.followTrajectory(trajShoot);
        sleep(300);
        shoot(3,false);
        bot.outtakeMotor.setVelocity(0);
        bot.followTrajectory(putAwayWobble10);
        servoWobble.open();
        sleep(200);
        bot.wobbleMotor.setTargetPosition(300);
        bot.wobbleMotor.setPower(0.2);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(300);
        bot.followTrajectory(returnBack);
        bot.followTrajectory(takeWobble2);
        servoWobble.close();
        sleep(300);
        bot.followTrajectory(putAwayWobble2);
        servoWobble.open();
        sleep(100);
        bot.wobbleMotor.setTargetPosition(0);
        bot.wobbleMotor.setPower(0.2);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            sleep(600);

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