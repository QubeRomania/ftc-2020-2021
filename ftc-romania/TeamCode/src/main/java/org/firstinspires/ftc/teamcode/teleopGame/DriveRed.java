package org.firstinspires.ftc.teamcode.teleopGame;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp
public class DriveRed extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(0);

    SampleMecanumDrive drive;

    //variabile outtake
    double outtakePower = 0;
    double basePowerOuttake = 1300;
    double powerShotPower = 1220;
    double powerUnit = 10;
    Boolean cheieOuttakeUp = Boolean.FALSE;
    Boolean cheieOuttakeDown = Boolean.FALSE;
    Boolean cheieOutake = Boolean.FALSE;
    Boolean okOuttake = Boolean.FALSE;
    Boolean cheiePowershot = Boolean.FALSE;
    Boolean okPowershot = Boolean.FALSE;
    double lastPower = 0;
    //outtake pid coefficients
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(60, 0, 14, 14.5); // 70,0,15,14 -- 60,0,14,14.5

    //variabile cutie
    servo_block servoBlock = new servo_block();
    Boolean boxIsUp = Boolean.FALSE;
    Boolean servoBlockUp = Boolean.FALSE;
    Boolean servoBlockDown = Boolean.FALSE;
    Boolean ok = Boolean.FALSE;
    Boolean cheie = Boolean.FALSE;
    double posCutie = 0.47;

    //variabile intake
    Boolean cheieIntake = Boolean.FALSE;
    Boolean isRunning = Boolean.FALSE;
    double intakePower = 0;

    //variabile piston
    servo_piston servoPiston = new servo_piston();
    Boolean cheiePiston = Boolean.FALSE;
    int pistonRelease = -165;
    int pistonClose = 0;
    double pistonPower = 0.4;

    //variabile perete
    servo_perete servoPerete = new servo_perete();
    Boolean pereteUp = Boolean.FALSE;
    Boolean pereteDown = Boolean.FALSE;
    Boolean cheiePerete = Boolean.FALSE;
    Boolean isOkPerete = Boolean.FALSE;
    double posPerete = 0.02;

    //variabile motor wobble
    Boolean cheieWobbleM = Boolean.FALSE;
    int wobbleRelease = -870;
    int wobbleClose = -300;
    Boolean isOpenedM = Boolean.FALSE;
    double wobblePower = 0.2;
    Boolean cheieFailSafe = Boolean.FALSE;
    Boolean isChanged = Boolean.FALSE;

    //variabile servo wobble
    servo_wobble servoWobble = new servo_wobble();
    Boolean cheieWobbleS = Boolean.FALSE;
    Boolean isOpened = Boolean.FALSE;

    Vector2d towerVector = new Vector2d(125,26);

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() {
        // Initialize custom cancelable SampleMecanumDrive class
        drive = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        servoPerete.close();
        servoPiston.initPiston(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outake", outtakePower);
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Cutie",posCutie);
        telemetry.addData("Velocity", drive.outtakeMotor.getVelocity());
        telemetry.update();

        drive.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Cutie",posCutie);
            telemetry.addData("Velocity", outtakePower);
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    //control sasiu
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.right_stick_y,
                                    -gamepad1.right_stick_x,
                                    -gamepad1.left_stick_x
                            )
                    );

                    if (gamepad1.y) {
                        targetAngle = Math.atan2(-poseEstimate.getY() + towerVector.getY(), -poseEstimate.getX() + towerVector.getX());

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    else if(gamepad2.right_bumper)
                    {
                        shoot(3);

                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    else if(gamepad1.right_bumper)
                    {
                        //powershot-uri automate
                        resetPositionLine();
                        servoBlock.open();
                        Trajectory powershot1 = drive.trajectoryBuilder(new Pose2d(63,0,0))
                                .lineToLinearHeading(new Pose2d(63.5,-26, Math.toRadians(-1)))
                                .build();

                        drive.outtakeMotor.setVelocity(powerShotPower);

                        drive.followTrajectory(powershot1);
                        shoot(1);
                        sleep(400);
                        drive.turn(Math.toRadians(-6));
                        shoot(1);
                        sleep(400);
                        drive.turn(Math.toRadians(-5));
                        shoot(1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

            intakeController();
            outtakeController();
            pistonController();
            servoBlockController();
            servoWobbleController();
            wobbleMotorController();
            pereteController();

            //Power To Intake
            drive.intakeMotor.setPower(intakePower);
            //Power To Outtake
            drive.outtakeMotor.setVelocity(outtakePower);

            dashboardTelemetry.addData("valoaree", drive.outtakeMotor.getVelocity());
            dashboardTelemetry.update();
        }
    }

    void pistonController()
    {
        if(gamepad2.left_bumper && !cheiePiston)
        {
            movePiston(0);
            cheiePiston = true;
        }
        if(!gamepad2.left_bumper)
            cheiePiston = false;

    }

    void servoBlockController()
    {
        if(intakePower>0) {
            //if intake is running, close the cage
            servoBlock.close();
            boxIsUp = false;
        }
        else {
            //toggle for cage (up/down)
            if (gamepad2.a && !cheie) {
                boxIsUp = !boxIsUp;
                cheie = !cheie;
                ok = true;
            }
            if (!gamepad2.a) {
                cheie = false;
            }
            if(ok) {
                if (boxIsUp)
                    servoBlock.open();
                else
                    servoBlock.close();
            }
        }

        //move one position up or down
        if (gamepad2.dpad_right) {
            if (!servoBlockUp) {
                posCutie += 0.01;
                if (posCutie > 0 && posCutie < 1)
                    servoBlock.setServoPositions(posCutie);
                servoBlockUp = true;
                ok = false;
            }
        }
        else
            servoBlockUp = false;

        if (gamepad2.dpad_left) {
            if (!servoBlockDown) {
                posCutie -= 0.01;
                if (posCutie > 0 && posCutie < 1)
                    servoBlock.setServoPositions(posCutie);
                servoBlockDown = true;
                ok = false;
            }
        }
        else
            servoBlockDown = false;

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
            //open wobble
            moveWobble(wobbleRelease,wobblePower,0);
        }
        else
        {
            //close wobble
            moveWobble(wobbleClose,wobblePower,0);
        }

        if(gamepad1.y && !cheieFailSafe)
        {
            isChanged = !isChanged;
            cheieFailSafe = !cheieFailSafe;
        }
        if(!gamepad1.y)
            cheieFailSafe = false;
        if(isChanged)
        {
            if(wobbleRelease != 600) {
                wobbleRelease = 600;
                wobbleClose = 100;
            }
            else
            {
                wobbleRelease = -870;
                wobbleClose = -300;
            }
        }
    }

    void intakeController()
    {
        //left trigger makes the intake turn reverse, else turn with trigger
            intakePower = gamepad2.right_trigger*0.8 - gamepad2.left_trigger*0.8;
    }

    void outtakeController()
    {
        //toggle b to turn outtake motor on/off
        if (gamepad2.b && !cheieOutake) {
            cheieOutake = !cheieOutake;
            okOuttake = !okOuttake;
        }
        if (!gamepad2.b) {
            cheieOutake = false;
        }
        if (okOuttake) {
                outtakePower = basePowerOuttake;
                lastPower = outtakePower;
        }
        else {
            outtakePower = 0;
            lastPower= 0;
        }

        if(gamepad2.x && !cheiePowershot){
            cheiePowershot = !cheiePowershot;
            okPowershot = !okPowershot;
        }
        if(!gamepad2.x)
        {
            cheiePowershot = false;
        }
        if(okPowershot)
        {
            lastPower = outtakePower;
            outtakePower = powerShotPower;
        }
        else
            outtakePower = lastPower;

        //dpad gives more or less power to outtake motor
        if(gamepad2.dpad_up) {
            if (!cheieOuttakeUp) {
                basePowerOuttake += powerUnit;
                cheieOuttakeUp = true;
            }
        }
        else
            cheieOuttakeUp = false;
        if(gamepad2.dpad_down) {
            if (!cheieOuttakeDown) {
                basePowerOuttake -= powerUnit;
                cheieOuttakeDown = true;
            }
        }
        else
            cheieOuttakeDown = false;
    }

    void pereteController()
    {
        //toggle on/off perete
        if (gamepad2.y && !cheiePerete) {
            isOkPerete = !isOkPerete;
            cheiePerete = !cheiePerete;
        }
        if (!gamepad2.y) {
            cheiePerete = false;
        }
        if (isOkPerete)
            servoPerete.setServoPositions(0.69);
        else
            servoPerete.setServoPositions(0.02);

        //move up/down one position
        if (gamepad1.dpad_right)
            if(!pereteUp)
            {
                posPerete+= 0.01;
                if(posPerete>0 && posPerete<1)
                    servoPerete.setServoPositions(posPerete);
                pereteUp = true;
            }
            else {}
        else
            pereteUp = false;

        if (gamepad1.dpad_left)
            if(!pereteDown)
            {
                posPerete -= 0.01;
                if(posPerete>0 && posPerete<1)
                    servoPerete.setServoPositions(posPerete);
                pereteDown = true;
            }
            else {}
        else
            pereteDown = false;
    }

    private void shoot(int rings)
    {
        for(int i=1;i<=rings;++i)
        {
            movePiston(150);
        }
    }

    void resetPositionLine()
    {
        drive.setPoseEstimate(new Pose2d(63,0,0));
    }

    void movePiston(int timeToSleep)
    {
        servoPiston.open();
        sleep(150);
        servoPiston.close();
        sleep(timeToSleep);
    }

    void moveWobble(int position, double power, int timeToSleep)
    {
        drive.wobbleMotor.setPower(power);
        drive.wobbleMotor.setTargetPosition(position);
        drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(timeToSleep);
    }
}