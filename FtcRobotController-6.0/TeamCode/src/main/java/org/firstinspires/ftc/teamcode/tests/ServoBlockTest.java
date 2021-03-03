package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

@TeleOp
public class ServoBlockTest extends LinearOpMode {
    servo_piston servoPiston = new servo_piston();
    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();

    @Override
    public void runOpMode()
    {
        servoPiston.initPiston(hardwareMap);
        servoBlock.initBlock(hardwareMap);
        servoWobble.initWobble(hardwareMap, true);
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x)
                servoBlock.open();
            else
                servoBlock.close();
        }
    }
}