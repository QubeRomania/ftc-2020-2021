package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class Autotest0Discuri extends LinearOpMode {

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

        Trajectory goForward = bot.trajectoryBuilder(putAwayWobble2.end())
                .splineTo(new Vector2d(117,-30), Math.toRadians(90))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory takeRings = bot.trajectoryBuilder(goForward.end())
                .addTemporalMarker(2,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .strafeTo(new Vector2d(118,10))
                .splineTo(new Vector2d(60,-10),Math.toRadians(-10))
                .build();

        Trajectory park = bot.trajectoryBuilder(takeRings.end())
                .lineTo(new Vector2d(70,0))
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
        bot.followTrajectory(takeWobble2);
        servoWobble.close();
        sleep(300);
        bot.followTrajectory(putAwayWobble2);
        servoWobble.open();
        sleep(100);
        bot.wobbleMotor.setTargetPosition(0);
        bot.wobbleMotor.setPower(0.2);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(300);
        bot.followTrajectory(goForward);
        bot.followTrajectory(takeRings);
        servoBlock.open();
        sleep(600);
        shoot(3,false);
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