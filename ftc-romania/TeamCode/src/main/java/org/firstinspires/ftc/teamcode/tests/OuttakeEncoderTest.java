package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

@TeleOp
public class OuttakeEncoderTest extends LinearOpMode {

    DcMotorEx outtakeMotor = null;
    public double HIGH_VELO = 100;

    @Override
    public void runOpMode(){
        outtakeMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            outtakeMotor.setVelocity(Math.min(gamepad2.left_trigger*2000, HIGH_VELO));
        }
    }
}