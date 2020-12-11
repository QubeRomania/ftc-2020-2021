package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public final class Hardware(hwMap: HardwareMap) {

    val motors = DriveMotors(hwmap);
    val intake = Intake(hwMap);

    public void stop()
    {
        motors.stop();
        intake.stop();
    }

}
