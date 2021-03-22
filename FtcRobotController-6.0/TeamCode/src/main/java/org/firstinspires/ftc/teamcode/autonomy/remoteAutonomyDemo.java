package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.util.Arrays;
import java.util.List;
import java.util.Vector;

@Autonomous
public class remoteAutonomyDemo extends LinearOpMode {

    public SampleMecanumDrive bot;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //developer key
    private static final String VUFORIA_KEY =
            "AfEZkiL/////AAABmaamsSiv9Ekeqgkncg9w6UKHCKU6c+yfO47f26KKNNRR5bu3Dqtk1794PqGUq3NuWygJPWjhsUkSBbDXsvzWfpHhGvZPx+TII7Io4o7hB8uAul0lxS1eywdu1374gI74XkwUD3rp08eW1EGW8nSrMXrwQgpx4ivWP7eVAFRGtciEYZ6+XC/tK0R9csWhRalAw1fcgFHJDFeO6NK3xgk01vVAItO3GRXjzEim9um6iWC70s67xCRFM+s2j+0oVCVyop5aPZ71Sn7k6wcSGW+eAgVtfNRslSBhEkUXaH0ThS6QaCeNhC4F44kuMpwMlrIwxZiiHJX1muex8zfcGN8alM+b67q8sJ8kO+QY5ePgMkxQ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public double zona = 0;

    servo_piston servoPiston = new servo_piston();
    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();

    @Override
    public void runOpMode()
    {
        bot = new SampleMecanumDrive(hardwareMap);
        servoPiston.initPiston(hardwareMap);
        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap,false);

        initVuforia();
        initTfod();

        if(tfod != null)
        {
            tfod.activate();
        }

        while(!isStarted())
        {
            if (tfod != null) {
                tfod.setClippingMargins(5,5,5,5);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("rings", "0");
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                zona = 1.0;
                                telemetry.addData("rings", "1");
                            } else if (recognition.getLabel().equals("Quad")) {
                                zona = 4.0;
                                telemetry.addData("rings", "4");
                            } else {
                                zona = 0.0;
                            }
                        }
                    }
                }
            }
        }

        waitForStart();

        tfod.shutdown();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive())
        {
            if(zona == 0)
            {
                Trajectory trajPowershot = bot.trajectoryBuilder(new Pose2d())
                        .addTemporalMarker(0.5, () -> {
                            bot.outtakeMotor.setPower(0.9);
                        })
                        .splineTo(new Vector2d(63, 18), 0)
                        .build();
                Trajectory trajPowershot2 = bot.trajectoryBuilder(trajPowershot.end())
                        .strafeTo(new Vector2d(63, 12))
                        .addTemporalMarker(0, () -> {
                            servoBlock.close();
                            servoBlock.open();
                        })
                        .build();
                Trajectory trajPowershot3 = bot.trajectoryBuilder(trajPowershot2.end())
                        .strafeTo(new Vector2d(63, 6))
                        .addTemporalMarker(0, () -> {
                            servoBlock.close();
                            servoBlock.open();
                        })
                        .build();
                Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot3.end())
                        .addTemporalMarker(0, () -> {
                            bot.outtakeMotor.setPower(0);
                            bot.wobbleMotor.setTargetPosition(-900);
                            bot.wobbleMotor.setPower(-0.15);
                            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(72, -15), -Math.toRadians(90))
                        .build();
                Trajectory returnToBase = bot.trajectoryBuilder(putAwayWobble1.end(), true)
                        .splineTo(new Vector2d(45, -14), 0)
                        .build();

                Trajectory takeWobble2 = bot.trajectoryBuilder(returnToBase.end())
                        .strafeTo(new Vector2d(30, -13))
                        .build();

                Trajectory putAwayWobble2 = bot.trajectoryBuilder(takeWobble2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .addTemporalMarker(2, () -> {
                            bot.wobbleMotor.setTargetPosition(-900);
                            bot.wobbleMotor.setPower(-0.2);
                            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(60, -23), 0)
                        .build();
                Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                        .back(3)
                        .splineToConstantHeading(new Vector2d(63, -9), 0)
                        .build();

                bot.followTrajectory(trajPowershot);
                sleep(100);
                shoot(1, false);
                bot.followTrajectory(trajPowershot2);
                shoot(1, true);
                bot.followTrajectory(trajPowershot3);
                shoot(1, false);

                bot.followTrajectory(putAwayWobble1);
                servoWobble.close();
                sleep(100);

                bot.followTrajectory(returnToBase);

                bot.followTrajectory(takeWobble2);
                servoWobble.open();
                sleep(600);
                bot.wobbleMotor.setTargetPosition(-300);
                bot.wobbleMotor.setPower(0.3);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                bot.turn(Math.toRadians(180));

                bot.followTrajectory(putAwayWobble2);

                servoWobble.close();
                sleep(200);
                bot.followTrajectory(parkRobot);
                stop();
            }
            else if(zona == 1)
            {
                Trajectory trajPowershot = bot.trajectoryBuilder(new Pose2d())
                        .addTemporalMarker(0.5, () -> {
                            bot.outtakeMotor.setPower(1);
                        })
                        .splineTo(new Vector2d(63, 18), 0)
                        .build();
                Trajectory trajPowershot2 = bot.trajectoryBuilder(trajPowershot.end())
                        .strafeTo(new Vector2d(63, 12))
                        .addTemporalMarker(0, () -> {
                            servoBlock.close();
                            servoBlock.open();
                        })
                        .build();
                Trajectory trajPowershot3 = bot.trajectoryBuilder(trajPowershot2.end())
                        .strafeTo(new Vector2d(63, 6))
                        .addTemporalMarker(0, () -> {
                            servoBlock.close();
                            servoBlock.open();
                        })
                        .build();
                Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowershot3.end())
                        .addTemporalMarker(0, () -> {
                            bot.outtakeMotor.setPower(0);
                            bot.wobbleMotor.setTargetPosition(-900);
                            bot.wobbleMotor.setPower(-0.15);
                            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .splineTo(new Vector2d(80, 4), 0)
                        .build();
                Trajectory returnToBase = bot.trajectoryBuilder(putAwayWobble1.end(), true)
                        .addTemporalMarker(0,()->{
                            servoBlock.close();
                        })
                        .splineTo(new Vector2d(60, 0), 0)
                        .build();

                Trajectory takeWobble2 = bot.trajectoryBuilder(returnToBase.end())
                        .strafeTo(new Vector2d(30, -12))
                        .build();

                Trajectory putAwayWobble2 = bot.trajectoryBuilder(takeWobble2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .addTemporalMarker(0, () -> {
                            bot.outtakeMotor.setPower(1);
                            bot.wobbleMotor.setTargetPosition(-900);
                            bot.wobbleMotor.setPower(-0.1);
                            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .addTemporalMarker(1.5,() -> {
                            shoot(1,false);
                        })
                        .splineTo(new Vector2d(80, -10), 0)
                        .build();
                Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                        .back(3)
                        .lineTo(new Vector2d(63, -10))
                        .build();

                bot.intakeMotor.setPower(1);

                bot.followTrajectory(trajPowershot);
                sleep(100);
                shoot(1, false);
                bot.followTrajectory(trajPowershot2);
                shoot(1, true);
                bot.followTrajectory(trajPowershot3);
                shoot(1, false);

                bot.followTrajectory(putAwayWobble1);
                servoWobble.close();
                sleep(100);

                bot.followTrajectory(returnToBase);

                bot.followTrajectory(takeWobble2);
                servoWobble.open();
                sleep(600);
                bot.wobbleMotor.setTargetPosition(-300);
                bot.wobbleMotor.setPower(0.3);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                bot.turn(Math.toRadians(180));

                servoBlock.open();

                bot.followTrajectory(putAwayWobble2);

                bot.outtakeMotor.setPower(0);

                servoWobble.close();
                sleep(200);
                bot.followTrajectory(parkRobot);
                stop();
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            servoPiston.open();
            sleep(600);
            servoPiston.close();
            sleep(600);
            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(100);
            }
        }
    }
}
