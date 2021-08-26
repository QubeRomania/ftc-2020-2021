package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;

@Config
@TeleOp
//@Disabled
public class PidTest extends LinearOpMode {

    // our DC motor.

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Boolean cheie = Boolean.FALSE;

    DcMotorEx outtakeMotor;
    servo_piston servoPiston = new servo_piston();

    public static double NEW_P = 0; //40
    public static double NEW_I = 0; //20
    public static double NEW_D = 0; //25
    public static double NEW_F = 0;
    public static double valoare = 1310;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        outtakeMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        servoPiston.initPiston(hardwareMap);

        Gamepad gp1 = gamepad1;
        Gamepad gp2 = gamepad2;

        servo_block servoBlock = new servo_block();
        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        // wait for start command.
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDFCoefficients pidModified = outtakeMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {

            if (gp1.left_bumper && !cheie){
                outtakeMotor.setVelocity(valoare);
                shoot(3);
            }
            if (!gp1.left_bumper){
                cheie = false;
            }

            if(gp1.right_bumper)
                outtakeMotor.setVelocity(valoare);


            dashboardTelemetry.addData("valoaree", outtakeMotor.getVelocity());
            dashboardTelemetry.update();
        }
    }
    private void shoot(int rings)
    {
        for(int i=1;i<=rings;++i)
        {
            movePiston(150);
        }
    }

    void movePiston(int timeToSleep)
    {
        servoPiston.open();
        sleep(150);
        servoPiston.close();
        sleep(timeToSleep);
    }
}