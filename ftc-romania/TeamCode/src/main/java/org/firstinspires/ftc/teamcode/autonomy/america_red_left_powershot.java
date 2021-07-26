package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.teleopGame.augmentedDrive.PoseStorage;

import java.util.List;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class america_red_left_powershot extends LinearOpMode {

    public SampleMecanumDrive bot;
    public double powershotPower = 1450;
    public double towerPower = 1620;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //developer key
    private static final String VUFORIA_KEY =
            "AfEZkiL/////AAABmaamsSiv9Ekeqgkncg9w6UKHCKU6c+yfO47f26KKNNRR5bu3Dqtk1794PqGUq3NuWygJPWjhsUkSBbDXsvzWfpHhGvZPx+TII7Io4o7hB8uAul0lxS1eywdu1374gI74XkwUD3rp08eW1EGW8nSrMXrwQgpx4ivWP7eVAFRGtciEYZ6+XC/tK0R9csWhRalAw1fcgFHJDFeO6NK3xgk01vVAItO3GRXjzEim9um6iWC70s67xCRFM+s2j+0oVCVyop5aPZ71Sn7k6wcSGW+eAgVtfNRslSBhEkUXaH0ThS6QaCeNhC4F44kuMpwMlrIwxZiiHJX1muex8zfcGN8alM+b67q8sJ8kO+QY5ePgMkxQ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public double zona = 0;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();
    servo_perete servoPerete = new servo_perete();

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);

        initVuforia();
        initTfod();

        if(tfod != null)
        {
            tfod.activate();
            tfod.setZoom(4.0, 16.0/9.0);
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
                        zona = 0.0;
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
                                telemetry.addData("rings", "0");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(powershotPower);
                })
                .splineTo(new Vector2d(63, 7), Math.toRadians(-1))
                .build();

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(trajShoot.end())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToLinearHeading(new Pose2d(80,-25,Math.toRadians(100)))
                .build();

        Trajectory park = bot.trajectoryBuilder(putAwayWobble10.end())
                .lineToLinearHeading(new Pose2d(70,0,Math.toRadians(0)))
                .build();

        Trajectory goToRing = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .splineTo(new Vector2d(30,-17),0)
                .build();

        Trajectory goShoot = bot.trajectoryBuilder(goToRing.end())
                .splineTo(new Vector2d(63.5,-5),Math.toRadians(-15))
                .build();

        Trajectory putAwayWobble11 = bot.trajectoryBuilder(trajShoot.end())
                .lineToLinearHeading(new Pose2d(83,-11.5,Math.toRadians(180)))
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        Trajectory takeRing1 = bot.trajectoryBuilder(putAwayWobble11.end())
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                })
                .lineTo(new Vector2d(30,-17))
                .build();

        Trajectory shoot1 = bot.trajectoryBuilder(takeRing1.end())
                .lineToLinearHeading(new Pose2d(62,-17,Math.toRadians(-2)))
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .build();

        Trajectory park1 = bot.trajectoryBuilder(shoot1.end())
                .lineToLinearHeading(new Pose2d(70,0,Math.toRadians(0)))
                .build();

        Trajectory putAwayWobble14 = bot.trajectoryBuilder(trajShoot.end())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    servoPerete.close();
                })
                .lineToLinearHeading(new Pose2d(123,-24,Math.toRadians(90)))
                .build();

        Trajectory goToRings = bot.trajectoryBuilder(putAwayWobble14.end())
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                })
                .lineToLinearHeading(new Pose2d(30,-18, Math.toRadians(180)))
                .build();

        Trajectory takeRings = bot.trajectoryBuilder(goToRings.end())
                .lineTo(new Vector2d(28,-18))
                .build();

        Trajectory shootRings4 = bot.trajectoryBuilder(takeRings.end())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .lineToLinearHeading(new Pose2d(62,-17,Math.toRadians(-3)))
                .build();

        Trajectory park4 = bot.trajectoryBuilder(shootRings4.end())
                .lineTo(new Vector2d(70,0))
                .build();

        waitForStart();

        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.pistonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tfod.shutdown();

        if (isStopRequested()) return;

        if(zona == 0.0) {
            bot.followTrajectory(trajShoot);
            servoPerete.open();
            sleep(200);
            shoot(1, false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.outtakeMotor.setVelocity(0);
            bot.followTrajectory(putAwayWobble10);
            servoWobble.open();
            sleep(200);
            bot.wobbleMotor.setTargetPosition(0);
            bot.wobbleMotor.setPower(0.2);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            bot.followTrajectory(park);
        }
        else if(zona == 1.0) {
            bot.followTrajectory(trajShoot);
            servoPerete.open();
            sleep(200);
            shoot(1, false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.outtakeMotor.setVelocity(0);
            bot.followTrajectory(putAwayWobble11);
            sleep(200);
            servoWobble.open();
            sleep(500);
            servoBlock.close();
            sleep(200);
            bot.followTrajectory(takeRing1);
            bot.followTrajectory(shoot1);
            servoBlock.open();
            servoPerete.open();
            sleep(600);
            shoot(1,false);
            bot.wobbleMotor.setTargetPosition(0);
            bot.wobbleMotor.setPower(0.2);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            bot.followTrajectory(park1);

        }
        else
        {
            bot.followTrajectory(trajShoot);
            servoPerete.open();
            sleep(200);
            shoot(1, false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.turn(Math.toRadians(-6));
            shoot(1,false);
            bot.outtakeMotor.setVelocity(0);
            bot.followTrajectory(putAwayWobble14);
            servoWobble.open();
            sleep(500);
            servoBlock.close();
            bot.followTrajectory(goToRings);
            bot.followTrajectory(takeRings);
            bot.followTrajectory(shootRings4);
            servoBlock.open();
            sleep(500);
            shoot(3,false);
            bot.wobbleMotor.setTargetPosition(0);
            bot.wobbleMotor.setPower(0.2);
            bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(300);
            bot.followTrajectory(park4);
        }
        PoseStorage.currentPose = bot.getPoseEstimate();

        stop();

    }
    private void shoot(int rings,boolean boxDown)
    {
        for(int i=1;i<=rings;++i)
        {
            bot.pistonMotor.setPower(0.3);
            bot.pistonMotor.setTargetPosition(-170);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(380);
            bot.pistonMotor.setTargetPosition(0);
            bot.pistonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(600);

            if(boxDown)
            {
                servoBlock.close();
                sleep(100);
                servoBlock.open();
                sleep(200);
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