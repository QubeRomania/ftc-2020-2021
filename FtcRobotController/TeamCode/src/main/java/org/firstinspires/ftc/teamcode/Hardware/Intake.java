package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.MotorPower
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(hwMap: HardwareMap){
    private DcMotor intakeMotor = hwmap.dcMotor.get("intakeMotor");

    public void setIntakeMotor(double power)
    {
        intakeMotor.setPower(power)e;
    }

    public void stopIntake()
    {
        setIntakePower(0.0);
    }

    public void stop()
    {
        stopIntake();
    }

}