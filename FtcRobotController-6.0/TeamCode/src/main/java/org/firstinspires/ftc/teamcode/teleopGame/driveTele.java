package org.firstinspires.ftc.teamcode.teleopGame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;

@TeleOp
public class driveTele extends LinearOpMode {

    protected Hardware robot = new Hardware();

    private ElapsedTime runtime = new ElapsedTime();

    double RF,LF,RR,LR;
    double X1,Y1,X2,Y2;

    double Scale = 1.0;
    double motorMax = 1.0;

    double intakePower = 0;
    double outtakePower = 0;

    Boolean boxIsUp = Boolean.FALSE;
    Boolean cheie = Boolean.FALSE;
    Boolean cheiePiston = Boolean.FALSE;
    Boolean isRunning = Boolean.FALSE;
    Boolean cheieIntake = Boolean.FALSE;

    servo_piston feeder = new servo_piston();

    @Override
    public void runOpMode()
    {
        robot.initHardware(hardwareMap);
        feeder.initPiston(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake", "Power:" + intakePower);
            telemetry.update();

            //motor powers
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;
            intakePower = 0;

            Y1 = -gamepad1.right_stick_y * Scale;
            X1 = gamepad1.right_stick_x * Scale;
            Y2 = -gamepad1.left_stick_y * Scale;
            X2 = gamepad1.left_stick_x * Scale;

            // Forward/back movement
            LF += Y1;
            RF += Y1;
            LR += Y1;
            RR += Y1;

            // Side to side movement
            LF += X1;
            RF -= X1;
            LR -= X1;
            RR += X1;

            // Rotation movement
            LF += X2;
            RF -= X2;
            LR += X2;
            RR -= X2;

            // Clip motor power values to +-motorMax
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            if(gamepad1.x && !cheieIntake)
            {
                isRunning = !isRunning;
                cheieIntake = !cheieIntake;
            }
            if(!gamepad1.x)
            {
                cheieIntake = false;
            }
            if(isRunning)
            {
                intakePower = 1.0;
            }
            else
                intakePower = 0.0;

            outtakePower = gamepad1.left_trigger;

            //giving power
            robot.totalPower(LF,RF,LR,RR,intakePower, outtakePower);

            //servoPiston controller
            if(gamepad1.a)
            {
                feeder.open();
            }
            else
            {
                feeder.close();
            }

            //servoBox controller
            if(gamepad1.b && !cheie)
            {
                boxIsUp = !boxIsUp;
                cheie = !cheie;
            }
            if(!gamepad1.b)
            {
                cheie = false;
            }
            if(boxIsUp)
                robot.servoBox.setPosition(0.3);
            else
                robot.servoBox.setPosition(0.0);
        }
    }
}