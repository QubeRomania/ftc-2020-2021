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
        leftRear = initMotorWithoutEncoder("leftRear");
        rightrear = initMotorWithoutEncoder("rightRear");
    }

    public void initFrontMotors()
    {
        leftFront = initMotorWithoutEncoder("leftFront");
        rightFront = initMotorWithoutEncoder("rightFront");
    }

    public void initMotors()
    {
        initBackMotors();
        initFrontMotors();
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void backPower(double left, double right)
    {
        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        leftRear.setPower(left);
        rightRear.setPower(right);
    }

    public void frontPower(double left, double right)
    {
        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        leftFront.setPower(left);
        rightFront.setPower(right);
    }

    public void total(double left, double right)
    {
        backPower(left, right);
        frontPower(left, right);
    }

    public void stop()
    {
        total(0,0);
    }

}