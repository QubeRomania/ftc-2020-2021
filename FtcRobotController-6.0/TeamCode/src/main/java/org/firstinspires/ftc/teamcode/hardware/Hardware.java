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
    public DcMotor intakeMotor = null;
    public DcMotor outtakeMotor = null;
    public Servo servoBox = null;

    //other things
    private HardwareMap hwMap = null;

    //Init hardware
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
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initFrontMotors() {
        leftFrontMotor = initMotorWithoutEncoder("leftFront");
        rightFrontMotor = initMotorWithoutEncoder("rightFront");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initIntake()
    {
        intakeMotor = initMotorWithoutEncoder("intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initOuttake()
    {
        outtakeMotor = initMotorWithoutEncoder("outtakeMotor");
    }

    private Servo initServo(String name)
    {
        Servo servo = hwMap.servo.get(name);
        servo.setPosition(0);
        return servo;
    }

    public void initServos()
    {
        servoBox = initServo("servoBox");
    }

    public void initMotors() {
        initBackMotors();
        initFrontMotors();
        initIntake();
        initOuttake();
    }

    public void initHardware(HardwareMap map) {
        hwMap = map;
        initMotors();
        initServos();
    }

    //MotorsPower
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

    public void totalPower (double i, double j, double k, double l, double z, double w){
        frontPower(i, j);
        backPower(k, l);
        z = clamp(z,-1,1);
        intakeMotor.setPower(z);
        outtakeMotor.setPower(w);
    }

    public void stop()
    {
        totalPower(0,0,0,0,0,0);
    }
}