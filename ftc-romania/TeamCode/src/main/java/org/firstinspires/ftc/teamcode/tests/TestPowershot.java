package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestPowershot extends LinearOpMode {
    DcMotorEx outtakeMotor = null;
    public double HIGH_VELO = 10;
    Boolean ant = Boolean.FALSE;
    Boolean ant1 = Boolean.FALSE;

    @Override
    public void runOpMode(){
        outtakeMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Velocity", HIGH_VELO);
            telemetry.update();
            if(gamepad1.dpad_down && ant == false)
            {
                HIGH_VELO-=100;
            }
            ant = !ant;
            if(gamepad1.dpad_up && ant == false)
            {
                HIGH_VELO+=100;
            }
            ant1 = !ant1;

            outtakeMotor.setVelocity(HIGH_VELO);
        }
    }
}
