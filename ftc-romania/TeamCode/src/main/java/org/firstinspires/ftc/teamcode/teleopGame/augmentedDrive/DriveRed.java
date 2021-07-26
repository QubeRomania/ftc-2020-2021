package org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.opencv.core.Mat;

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

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector;
    // The heading we want the bot to end on for targetA
    double targetAHeading;

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(0);


    SampleMecanumDrive drive;

    double Scale = 1.0;
    double motorMax = 1.0;

    double intakePower = 0;
    double outtakePower = 0;
    double wobblePower = 0;
    double pistonPower = 0;
    double copieOutake = 0;
    double posCutie = 0.47;
    double posPerete = 0.02;
    double posPerete1 = 0.69;
    double basePowerOuttake = 1625;

    public double HIGH_VELO = 1600;

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
    Boolean servoBlockUp = Boolean.FALSE;
    Boolean servoBlockDown = Boolean.FALSE;
    Boolean ok = Boolean.FALSE;
    Boolean okOuttake = Boolean.FALSE;
    Boolean pereteUp = Boolean.FALSE;
    Boolean pereteDown = Boolean.FALSE;
    Boolean cheiePerete = Boolean.FALSE;
    Boolean isOkPerete = Boolean.FALSE;
    Boolean cheieOuttakeUp = Boolean.FALSE;
    Boolean cheieOuttakeDown = Boolean.FALSE;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();
    servo_perete servoPerete = new servo_perete();

    Vector2d towerVector = new Vector2d(125,26);

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(52, 0, 4.6, 14);
    private VoltageSensor batteryVoltageSensor;

    double powerShotPower = 1440;

    BNO055IMU imu;

    @Override
    public void runOpMode() {
        // Initialize custom cancelable SampleMecanumDrive class
        drive = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        servoPerete.close();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outake", outtakePower);
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Cutie",posCutie);
        telemetry.addData("Velocity", drive.outtakeMotor.getVelocity());
        telemetry.update();

        drive.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                        shoot(3,false);

                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    else if(gamepad1.right_bumper)
                    {
                        resetPositionLine();
                        servoBlock.open();
                        Trajectory powershot1 = drive.trajectoryBuilder(new Pose2d(63,0,0))
                                .lineToLinearHeading(new Pose2d(63.5,-26, Math.toRadians(-1)))
                                .build();

                        drive.outtakeMotor.setVelocity(powerShotPower);

                        drive.followTrajectory(powershot1);
                        shoot(1,false);
                        sleep(400);
                        drive.turn(Math.toRadians(-6));
                        shoot(1, false);
                        sleep(400);
                        drive.turn(Math.toRadians(-5));
                        shoot(1, false);

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



            //giving power
            drive.intakeMotor.setPower(intakePower);
            drive.outtakeMotor.setVelocity(outtakePower);
            drive.wobbleMotor.setPower(wobblePower);
        }
    }

    void pistonController()
    {
        if(gamepad2.left_bumper && !cheiePiston)
        {
            if(boxIsUp) {
                pistonPower = 0.40;
                drive.pistonMotor.setPower(pistonPower);
                drive.pistonMotor.setTargetPosition(-165);
                drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                drive.pistonMotor.setTargetPosition(0);
                drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cheiePiston = true;
            }
        }
        if(!gamepad2.left_bumper)
            cheiePiston = false;
    }

    void servoBlockController()
    {
        if(intakePower>0) {
            servoBlock.close();
            boxIsUp = false;
        }
        else {
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

        if (gamepad2.dpad_right)
            if(!servoBlockUp)
            {
                posCutie += 0.01;
                if(posCutie>0 && posCutie<1)
                    servoBlock.setServoPositions(posCutie);
                servoBlockUp = true;
                ok = false;
            }
            else {}
        else
            servoBlockUp = false;

        if (gamepad2.dpad_left)
            if(!servoBlockDown)
            {
                posCutie -= 0.01;
                if(posCutie>0 && posCutie<1)
                    servoBlock.setServoPositions(posCutie);
                servoBlockDown = true;
                ok = false;
            }
            else {}
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
            wobblePower = 0.2;
            drive.wobbleMotor.setTargetPosition(1000);
            drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            drive.wobbleMotor.setTargetPosition(200);
            drive.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if(!(gamepad2.x))
        {
            cheieIntake = false;
        }
        if(isRunning)
        {
            intakePower = -1;
        }
        else {
                intakePower = gamepad2.right_trigger;
        }
    }

    void outtakeController()
    {
        /*if(gamepad2.b && !cheieOutakeP)
        {
            isPowerShot = !isPowerShot;
            cheieOutakeP = !cheieOutakeP;
        }
        if(!gamepad2.b)
            cheieOutakeP = false;
        if(isPowerShot)
        {
            outtakePower = powerShotPower;
        }
        else
            outtakePower = Math.min(gamepad2.left_trigger*2000, HIGH_VELO);

         */
        /*if(gamepad2.b && !cheieOutake)
        {
            outtakePower = 1600;
            cheieOutake = true;
        }
         */
        if (gamepad2.b && !cheieOutake) {
            cheieOutake = !cheieOutake;
            okOuttake = !okOuttake;
        }
        if (!gamepad2.b) {
            cheieOutake = false;
        }
        if (okOuttake)
            outtakePower = basePowerOuttake;
        else
            outtakePower = 0;

        if(gamepad2.dpad_up)
            if(!cheieOuttakeUp)
            {
                basePowerOuttake += 25;
                cheieOuttakeUp = true;
            }
            else
            {

            }
        else
        {
            cheieOuttakeUp = false;
        }
        if(gamepad2.dpad_down)
            if(!cheieOuttakeDown)
            {
                basePowerOuttake -= 25;
                cheieOuttakeDown = true;
            }
            else{

            }
        else
        {
            cheieOuttakeDown = false;
        }
    }

    void pereteController()
    {
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

        if (gamepad2.dpad_right)
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

        if (gamepad2.dpad_left)
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

    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            drive.pistonMotor.setPower(0.3);
            drive.pistonMotor.setTargetPosition(-165);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(380);
            drive.pistonMotor.setTargetPosition(0);
            drive.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(i!=rings)
                sleep(400);

            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(200);
            }
        }
    }

    void resetPositionCorner()
    {
        drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    void resetPositionLine()
    {
        drive.setPoseEstimate(new Pose2d(63,0,0));
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
}