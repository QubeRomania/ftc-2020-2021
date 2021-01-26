package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public final class Hardware {
    //DC motors
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;

    //other things
    private HardwareMap hwMap = null;

    private DcMotor initMotorWithoutEncoder(String name) {
        DcMotor motor = hwMap.dcMotor.get(name);

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return motor;
    }

    private DcMotor initMotorWithEncoder(String name) {
        DcMotor motor = hwMap.dcMotor.get(name);

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return motor;
    }

    public void initBackMotors() {
        leftBackMotor = initMotorWithoutEncoder("leftRear");
        rightBackMotor = initMotorWithoutEncoder("rightRear");
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initFrontMotors() {
        leftFrontMotor = initMotorWithoutEncoder("leftFront");
        rightFrontMotor = initMotorWithoutEncoder("rightFront");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initMotors() {
        initBackMotors();
        initFrontMotors();
    }


    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void backPower (double left, double right){
        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        leftBackMotor.setPower(left);
        rightBackMotor.setPower(right);
    }

    public void frontPower (double left, double right){
        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        leftFrontMotor.setPower(left);
        rightFrontMotor.setPower(right);
    }

    public void totalPower (double i, double j){
        frontPower(i, j);
        backPower(i, j);
    }


    public void init(HardwareMap map) {
        hwMap = map;
    }

    public void stop()
    {
        totalPower(0,0);
    }

}