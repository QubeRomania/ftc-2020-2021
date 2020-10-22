package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public final class Hardware {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    private HardwareMap hwmap = null;

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

    public static final double MAX_LIMIT = 0.8;

    public static double clamp(double value, double min, double max)
    {
        return Math.max(min, Math.Min(value,max));
    }

    public void tractiuneSpate(double left, double right)
    {
        left = clamp(left,-1,1);
        right = clamp(right, -1, 1);

        leftBackMotor.setPower(left * MAX_LIMIT);
        rightBackMotor.setPower(right * MAX_LIMIT);
    }

    public void tractiuneFata(double left,double right)
    {
        left = clamp(left, -1, 1);
        right = clamp(right, -1, 1);

        leftFrontMotor.setPower(left * MAX_LIMIT);
        rightFrontMotor.setPower(right * MAX_LIMIT);
    }

    public void tractiuneIntegrala(double i, double j)
    {
        tractiuneFata(i,j);
        tractiuneSpate(i,j);

    }

    public void init(HardwareMap map)
    {
        hwmap = map;

    }


}
