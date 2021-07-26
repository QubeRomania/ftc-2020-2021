package org.firstinspires.ftc.teamcode.teleopGame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

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
    double wobblePower = 0;
    double pistonPower = 0;
    double copieOutake = 0;
    double posPerete = 0.0;

    Boolean boxIsUp = Boolean.FALSE;
    Boolean cheie = Boolean.FALSE;
    Boolean cheiePiston = Boolean.FALSE;
    Boolean isPistonOk = Boolean.FALSE;
    Boolean isRunning = Boolean.FALSE;
    Boolean cheieIntake = Boolean.FALSE;
    Boolean cheieOutake = Boolean.FALSE;
    Boolean isMax = Boolean.FALSE;
    Boolean precisionMode = Boolean.FALSE;
    Boolean cheiePrecision = Boolean.FALSE;
    Boolean isOpened = Boolean.FALSE;
    Boolean cheieWobbleS = Boolean.FALSE;
    Boolean isOpenedM = Boolean.FALSE;
    Boolean cheieWobbleM = Boolean.FALSE;
    Boolean isPowerShot = Boolean.FALSE;
    Boolean cheieOutakeP = Boolean.FALSE;
    Boolean cheieUp = Boolean.FALSE;
    Boolean cheieDown = Boolean.FALSE;
    Boolean goUp = Boolean.FALSE;
    Boolean goDown = Boolean.FALSE;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();
    servo_perete servoPerete = new servo_perete();

    @Override
    public void runOpMode()
    {
        robot.initHardware(hardwareMap);
        servoBlock.initBlock(hardwareMap);
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outake", outtakePower);
        telemetry.addData("Intake", intakePower);
        telemetry.update();

        robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake", "Power:" + intakePower);
            telemetry.addData("Outtake", outtakePower);
            telemetry.addData("Pozitie Perete",posPerete);
            telemetry.update();

            //motor powers
            // Reset speed variables
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;
            intakePower = 0;

            if(gamepad1.right_bumper && !cheiePrecision)
            {
                precisionMode = !precisionMode;
                cheiePrecision = !cheiePrecision;
            }
            if(!gamepad1.right_bumper)
                cheiePrecision = false;
            if(cheiePrecision)
                Scale = 0.5;
            else
                Scale = 1.0;

            chassisComponents();
            intakeController();
            outtakeController();
            pistonController();
            servoBlockController();
            servoWobbleController();
            wobbleMotorController();
            isPowerShotController();
            changePosPiston();

            //giving power
            robot.chassisPower(LF,RF,LR,RR);
            robot.intakeMotor.setPower(intakePower);
            robot.outtakeMotor.setPower(outtakePower);
            robot.wobbleMotor.setPower(wobblePower);
            servoPerete.setServoPositions(posPerete);
        }
    }

    void chassisComponents()
    {
        //left stick - movement
        Y1 = -gamepad1.right_stick_y * Scale; // invert so up is positive
        X1 = gamepad1.right_stick_x * Scale;
        Y2 = -gamepad1.left_stick_y * Scale; // Y2 is not used at present
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
    }

    void pistonController()
    {
        if(gamepad2.left_bumper && !cheiePiston)
        {
            pistonPower = 0.4;
            robot.pistonMotor.setPower(pistonPower);
            robot.pistonMotor.setTargetPosition(-170);
            robot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            robot.pistonMotor.setTargetPosition(0);
            robot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cheiePiston = true;
        }
        if(!gamepad2.left_bumper)
            cheiePiston = false;
    }

    void servoBlockController()
    {
        if(intakePower>0)
            servoBlock.close();
        else {
            if (gamepad2.a && !cheie) {
                boxIsUp = !boxIsUp;
                cheie = !cheie;
            }
            if (!gamepad2.a) {
                cheie = false;
            }
            if (boxIsUp)
                servoBlock.open();
            else
                servoBlock.close();
        }
    }

    void servoWobbleController()
    {
        if(gamepad1.a && !cheieWobbleS)
        {
            isOpened = !isOpened;
            cheieWobbleS = !cheieWobbleS;
        }
        if(!gamepad1.a)
            cheieWobbleS = false;
        if(isOpened)
            servoWobble.open();
        else
            servoWobble.close();
    }

    void wobbleMotorController()
    {
        if(gamepad1.b && !cheieWobbleM)
        {
            isOpenedM = !isOpenedM;
            cheieWobbleM = !cheieWobbleM;
        }
        if(!gamepad1.b)
            cheieWobbleM = false;
        if(isOpenedM) {
            wobblePower = 0.2;
            robot.wobbleMotor.setTargetPosition(700);
            robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            robot.wobbleMotor.setTargetPosition(0);
            robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobblePower = 0.2;
        }
    }

    void intakeController()
    {
        if(gamepad2.x && !cheieIntake)
        {
            isRunning = !isRunning;
            cheieIntake = !cheieIntake;
        }
        if(!gamepad2.x)
        {
            cheieIntake = false;
        }
        if(isRunning)
        {
            intakePower = 1.0;
        }
        else {
            if(gamepad2.right_trigger!=0)
            {
                intakePower = -gamepad2.right_trigger;
            }
            else
                intakePower = 0.0;
        }
    }

    void outtakeController()
    {
        if(gamepad2.y && !cheieOutake)
        {
            isMax = !isMax;
            cheieOutake = !cheieOutake;
        }
        if(!gamepad2.y)
            cheieOutake = false;
        if(isMax)
        {
            outtakePower = 1.0;
        }
        else
        {
            if(gamepad2.left_trigger!=0)
            {
                outtakePower = gamepad2.left_trigger;
            }
            else
                outtakePower = 0.0;
        }
    }

    void isPowerShotController()
    {
        if(gamepad2.b && !cheieOutakeP)
        {
            isPowerShot = !isPowerShot;
            cheieOutakeP = !cheieOutakeP;
        }
        if(!gamepad2.b)
            cheieOutakeP = false;
        if(isPowerShot)
        {
            outtakePower = 0.9;
        }
        else
            outtakePower = gamepad2.left_trigger - gamepad1.left_trigger;
    }

    void changePosPiston()
    {
        if(gamepad1.dpad_up && !cheieUp)
        {
            telemetry.addData("Ajuns",1);
            goUp = !goUp;
            cheieUp = !cheieUp;
        }
        if(!gamepad1.dpad_up)
            cheieUp = false;
        if(goUp)
        {
            telemetry.addData("Ajuns",2);
            posPerete += 0.02;
            if(posPerete>1)
                posPerete = 1;
        }

        if(gamepad1.dpad_down && !cheieDown)
        {
            goDown = !goDown;
            cheieDown = !cheieDown;
        }
        if(!gamepad1.dpad_down)
            cheieDown = false;
        if(goDown)
        {
            posPerete -= 0.02;
            if(posPerete<0)
                posPerete = 0;
        }
    }
}