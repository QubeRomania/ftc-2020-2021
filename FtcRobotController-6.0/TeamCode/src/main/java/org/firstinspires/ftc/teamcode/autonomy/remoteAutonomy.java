package org.firstinspires.ftc.teamcode.autonomy;

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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;

import java.util.List;
import java.util.Vector;

@Autonomous
public class remoteAutonomy extends LinearOpMode {

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
            telemetry.addData("Zona",zona);
            bot.updatePoseEstimate();
            Pose2d currentPose = bot.getPoseEstimate();

            telemetry.addData("POSE", "X = %.2f  Y = %.2f  H = %.1f", currentPose.getX(),
                    currentPose.getY(), Math.toDegrees(currentPose.getHeading()));


            if(zona == 0)
            {
                Trajectory trajPowerShot1 = bot.trajectoryBuilder(new Pose2d())
                        .forward(13)
                        .splineTo(new Vector2d(63,-8),0)
                        .build();
                Trajectory putWobble1 = bot.trajectoryBuilder(trajPowerShot1.end())
                        .strafeTo(new Vector2d(63,-22.5))
                        .build();
                /*Trajectory trajPowerShot3 = bot.trajectoryBuilder(trajPowerShot2.end())
                        .strafeTo(new Vector2d(63,4))
                        .build();

                 */
                Trajectory littleBack = bot.trajectoryBuilder(putWobble1.end())
                        .strafeTo(new Vector2d(55,-22.5))
                        .build();
                Trajectory returnToBase = bot.trajectoryBuilder(littleBack.end().plus(new Pose2d(0,0,-Math.toRadians(180))))
                        .splineTo(new Vector2d(45,-12),Math.toRadians(180))
                        .build();
                Trajectory takeWobble = bot.trajectoryBuilder(returnToBase.end())
                        .strafeTo(new Vector2d(30,-10))
                        .build();
                Trajectory putAwayWobble2 = bot.trajectoryBuilder(takeWobble.end().plus(new Pose2d(0,0,-Math.toRadians(180))))
                        .strafeTo(new Vector2d(57,-26))
                        .addTemporalMarker(0,()->{
                            bot.wobbleMotor.setTargetPosition(-900);
                            bot.wobbleMotor.setPower(-0.05);
                            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        })
                        .build();
                Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                        .back(3)
                        .splineToConstantHeading(new Vector2d(64,-5),0)
                        .build();

                //arunc discuri in powershot
                bot.followTrajectory(trajPowerShot1);
                bot.outtakeMotor.setPower(1);
                sleep(700);
                servoPiston.open();
                sleep(600);
                servoPiston.close();
                sleep(600);
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(500);
                servoPiston.open();
                sleep(600);
                servoPiston.close();
                sleep(600);
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(500);
                servoPiston.open();
                sleep(600);
                servoPiston.close();
                bot.outtakeMotor.setPower(0);

                bot.followTrajectory(putWobble1);

                //las wobble
                bot.wobbleMotor.setTargetPosition(-900);
                bot.wobbleMotor.setPower(-0.3);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1200);
                servoWobble.close();
                sleep(500);

                bot.followTrajectory(littleBack);

                bot.turn(Math.toRadians(180));

                bot.followTrajectory(returnToBase);

                bot.followTrajectory(takeWobble);

                servoWobble.open();
                sleep(500);
                bot.wobbleMotor.setTargetPosition(-300);
                bot.wobbleMotor.setPower(0.3);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                bot.turn(-Math.toRadians(180));

                bot.followTrajectory(putAwayWobble2);

                servoWobble.close();
                sleep(500);

                bot.followTrajectory(parkRobot);

                //iau wobble 2
                stop();
                /*
                bot.followTrajectory(putAwayWobble2);
                //las wobble
                bot.followTrajectory(parkRobot);

                 */
            }
            else
            {
                if(zona == 1)
                {
                    Trajectory goToRingF = bot.trajectoryBuilder(new Pose2d())
                            .strafeTo(new Vector2d(25,0))
                            .build();
                    Trajectory goToRingS = bot.trajectoryBuilder(goToRingF.end())
                            .strafeTo(new Vector2d(25,-2))
                            .addTemporalMarker(0, ()-> {
                                bot.outtakeMotor.setPower(1);
                            })
                            .build();
                    Trajectory takeRing = bot.trajectoryBuilder(goToRingS.end())
                            .strafeTo(new Vector2d(30,-2))
                            .addTemporalMarker(0,()-> {
                                bot.intakeMotor.setPower(1);
                            })
                            .build();
                    Trajectory trajPowerShot = bot.trajectoryBuilder(takeRing.end())
                            .splineTo(new Vector2d(63,-8),0)
                            .addTemporalMarker(0,()-> {
                                bot.outtakeMotor.setPower(1);
                            })
                            .build();
                    Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowerShot.end())
                            .splineTo(new Vector2d(80, 0), 0)
                            .addTemporalMarker(0, ()-> {
                                bot.outtakeMotor.setPower(0);
                                bot.intakeMotor.setPower(0);
                                bot.wobbleMotor.setTargetPosition(-900);
                                bot.wobbleMotor.setPower(-0.2);
                                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            })
                            .build();
                    Trajectory littleBack = bot.trajectoryBuilder(putAwayWobble1.end())
                            .strafeTo(new Vector2d(70,0))
                            .build();
                    Trajectory returnToBase = bot.trajectoryBuilder(littleBack.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .splineTo(new Vector2d(45, -11),Math.toRadians(180))
                            .build();
                    Trajectory littleFront = bot.trajectoryBuilder(returnToBase.end())
                            .strafeTo(new Vector2d(30,-11))
                            .build();
                    Trajectory putAwayWobble2 = bot.trajectoryBuilder(littleFront.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .splineTo(new Vector2d(72,-11),0)
                            .addTemporalMarker(2.5,() -> {
                                servoWobble.close();
                            })
                            .build();
                    Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                            .strafeTo(new Vector2d(70,-11))
                            .build();

                    bot.followTrajectory(goToRingF);
                    bot.followTrajectory(goToRingS);
                    bot.turn(Math.toRadians(2.5));
                    sleep(100);
                    servoPiston.open();
                    sleep(600);
                    servoPiston.close();
                    sleep(600);
                    bot.outtakeMotor.setPower(0);
                    servoBlock.close();
                    sleep(100);
                    bot.turn(-Math.toRadians(2.5));
                    bot.followTrajectory(takeRing);
                    bot.followTrajectory(trajPowerShot);

                    servoBlock.open();
                    sleep(100);

                    bot.turn(-Math.toRadians(1));

                    sleep(200);
                    servoPiston.open();
                    sleep(500);
                    servoPiston.close();
                    sleep(500);
                    servoPiston.open();
                    sleep(500);
                    servoPiston.close();
                    sleep(500);
                    servoPiston.open();
                    sleep(500);
                    servoPiston.close();

                    bot.turn(Math.toRadians(1));

                    bot.followTrajectory(putAwayWobble1);

                    servoWobble.close();
                    sleep(100);

                    bot.followTrajectory(littleBack);
                    bot.turn(Math.toRadians(180));
                    bot.followTrajectory(returnToBase);

                    bot.followTrajectory(littleFront);

                    servoWobble.open();
                    sleep(500);
                    bot.wobbleMotor.setTargetPosition(-300);
                    bot.wobbleMotor.setPower(0.3);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    bot.turn(-Math.toRadians(180));

                    bot.wobbleMotor.setTargetPosition(-900);
                    bot.wobbleMotor.setPower(-0.05);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    bot.followTrajectory(putAwayWobble2);
                    bot.followTrajectory(parkRobot);

                    stop();
                }
                else
                {
                    Trajectory trajS = bot.trajectoryBuilder(new Pose2d())
                            .strafeTo(new Vector2d(0,10))
                            .build();
                    Trajectory trajShoot = bot.trajectoryBuilder(trajS.end())
                            .forward(15)
                            .splineToConstantHeading(new Vector2d(63,-4),0)
                            .build();
                    /*Trajectory takeRings = bot.trajectoryBuilder(trajShoot.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .strafeTo(new Vector2d(40,-4))
                            .build();
                    Trajectory littleBack = bot.trajectoryBuilder(takeRings.end())
                            .strafeTo(new Vector2d(35,-4))
                            .build();
                    Trajectory trajShoot1 = bot.trajectoryBuilder(littleBack.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .splineTo(new Vector2d(63,-4),0)
                            .build();
                    Trajectory takeRest = bot.trajectoryBuilder(trajShoot1.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .strafeTo(new Vector2d(20,0))
                            .build();
                    Trajectory trajShoot2 = bot.trajectoryBuilder(takeRest.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                            .strafeTo(new Vector2d(63,-4))
                            .build();

                     */
                    Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajShoot.end())
                            .splineTo(new Vector2d(100, -22.5), 0)
                            .build();
                    Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble1.end())
                            .strafeTo(new Vector2d(70,-22.5))
                            .build();

                    bot.followTrajectory(trajS);

                    bot.followTrajectory(trajShoot);

                    servoBlock.open();
                    sleep(200);

                    bot.outtakeMotor.setPower(1);
                    sleep(700);
                    servoPiston.open();
                    sleep(600);
                    servoPiston.close();
                    sleep(600);
                    servoBlock.close();
                    sleep(100);
                    servoBlock.open();
                    sleep(300);
                    servoPiston.open();
                    sleep(600);
                    servoPiston.close();
                    sleep(600);
                    servoBlock.close();
                    sleep(100);
                    servoBlock.open();
                    sleep(300);
                    servoPiston.open();
                    sleep(600);
                    servoPiston.close();
                    bot.outtakeMotor.setPower(0);

                    bot.followTrajectory(putAwayWobble1);

                    bot.wobbleMotor.setTargetPosition(-900);
                    bot.wobbleMotor.setPower(-0.3);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    servoWobble.close();
                    sleep(500);

                    bot.followTrajectory(parkRobot);

                    stop();
                }
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
}
