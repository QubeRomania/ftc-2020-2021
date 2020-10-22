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

    public void initBackMotors()
    {
        leftBackMotor = hwmap.dcMotor.class("leftBackMotor");
        rightBackMotor = hwmap.dcMotor.class("rightBackMotor");
    }

    public void initFrontMotors()
    {
        leftFrontMotor = hwmap.dcMotor.class("leftFrontMotor");
        rightFrontMotor = hwmap.dcMotor.class("rightFrontMotor");
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

    
}
