package org.firstinspires.ftc.teamcode.teleopGame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp
public class driveTele extends LinearOpMode {

    protected Hardware robot = new Hardware();

    private ElapsedTime runtime = new ElapsedTime();

    double RF,LF,RR,LR;

    double X1,Y1,X2,Y2;

    double Scale = 0.5;
    double motorMax = 0.6;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            Y1 = -gamepad1.right_stick_y * Scale;
            X1 = gamepad1.right_stick_x * Scale;
            Y2 = -gamepad1.left_stick_y * Scale;
            X2 = gamepad1.left_stick_x * Scale;

            // Forward/back movement
            LF += Y1;
            RF += Y1;
            LR += Y1;
            RR += Y1;

            // Side to side movement
            LF += X1;
            RF -= X1;
            LR -= X1;
            RR += X1;

            // Rotation movement
            LF += X2;
            RF -= X2;
            LR += X2;
            RR -= X2;

            // Clip motor power values to +-motorMax
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            robot.leftFrontMotor.setPower(LF);
            robot.rightFrontMotor.setPower(RF);
            robot.leftBackMotor.setPower(LR);
            robot.rightBackMotor.setPower(RR);
        }
    }
}
