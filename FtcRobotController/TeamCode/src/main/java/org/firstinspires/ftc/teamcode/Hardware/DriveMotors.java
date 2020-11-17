package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.MotorPower
import org.firstinspires.ftc.robotcore.external.Telemetry

public final class DriveMotors(hwMap: HardwareMap){
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    private DcMotor initMotorWithoutEncoder(String name)
    {
        DcMotor motor = hwmap.dcMotor.get(name);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private DcMotor initMotorWithEncoder(String name)
    {
        DcMotor motor = hwmap.dcMotor.get(name);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        return motor;
    }

    public void initBackMotors()
    {
        leftBackMotor = initMotorWithoutEncoder("leftBackMotor");
        rightBackMotor = initMotorWithoutEncoder("rightBackMotor");
    }

    public void initFrontMotors()
    {
        leftFrontMotor = initMotorWithoutEncoder("leftFrontMotor");
        rightFrontMotor = initMotorWithoutEncoder("rightFrontMotor");
    }

    public void initMotors()
    {
        initBackMotors();
        initFrontMotors();
    }

}