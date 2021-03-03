package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.util.List;

@Autonomous
public class wobbleAutonomyTest extends LinearOpMode {

    public SampleMecanumDrive bot;

    public double zona = 0;

    servo_wobble servoWobble = new servo_wobble();

    @Override
    public void runOpMode()
    {
        bot = new SampleMecanumDrive(hardwareMap);
        servoWobble.initWobble(hardwareMap,false);

        waitForStart();

        while(opModeIsActive())
        {
            bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bot.wobbleMotor.setTargetPosition(-700);
            bot.wobbleMotor.setPower(0.3);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            servoWobble.close();
            sleep(1000);
            servoWobble.open();
            sleep(1000);
            bot.wobbleMotor.setTargetPosition(0);
            bot.wobbleMotor.setPower(0.3);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Wobble", "is Up again");
        }
    }
}
