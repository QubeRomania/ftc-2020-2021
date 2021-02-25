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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

    @Override
    public void runOpMode()
    {
        bot = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();

        if(tfod != null)
        {
            tfod.activate();
        }

        while(!isStarted())
        {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
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
                            } else if (recognition.getLabel().equals("Quad")) {
                                zona = 4.0;
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

        while(opModeIsActive())
        {
            bot.updatePoseEstimate();
            Pose2d currentPose = bot.getPoseEstimate();

            telemetry.addData("POSE", "X = %.2f  Y = %.2f  H = %.1f", currentPose.getX(),
                    currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

            if(zona == 0)
            {
                Trajectory trajPowerShot = bot.trajectoryBuilder(new Pose2d())
                        .strafeLeft(5)
                        .splineToConstantHeading(new Vector2d(63,20),0)
                        .build();
                Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowerShot.end())
                        .splineToConstantHeading(new Vector2d(115, -22.5), 0)
                        .build();
                Trajectory returnToBase = bot.trajectoryBuilder(putAwayWobble1.end())
                        .strafeTo(new Vector2d(0, -22.5))
                        .build();
                Trajectory putAwayWobble2 = bot.trajectoryBuilder(returnToBase.end())
                        .strafeTo(new Vector2d(115,-22.5))
                        .build();
                Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                        .strafeTo(new Vector2d(65,-22.5))
                        .build();

                bot.followTrajectory(trajPowerShot);
                //arunc discuri in powershot
                bot.followTrajectory(putAwayWobble1);
                //las wobble
                bot.followTrajectory(returnToBase);
                //iau wobble 2
                bot.followTrajectory(putAwayWobble2);
                //las wobble
                bot.followTrajectory(parkRobot);
            }
            else
            {
                if(zona == 1)
                {
                    Trajectory trajPowerShot = bot.trajectoryBuilder(new Pose2d())
                            .strafeLeft(5)
                            .splineToConstantHeading(new Vector2d(63,20),0)
                            .build();
                    Trajectory putAwayWobble1 = bot.trajectoryBuilder(trajPowerShot.end())
                            .splineToConstantHeading(new Vector2d(115, -22.5), 0)
                            .build();
                    Trajectory returnToBase = bot.trajectoryBuilder(putAwayWobble1.end())
                            .strafeTo(new Vector2d(0, -22.5))
                            .build();
                    Trajectory putAwayWobble2 = bot.trajectoryBuilder(returnToBase.end())
                            .strafeTo(new Vector2d(115,-22.5))
                            .build();
                    Trajectory parkRobot = bot.trajectoryBuilder(putAwayWobble2.end())
                            .strafeTo(new Vector2d(65,-22.5))
                            .build();


                    bot.followTrajectory(trajPowerShot);
                    //arunc discuri in powershot
                    bot.followTrajectory(putAwayWobble1);
                    //las wobble
                    bot.followTrajectory(returnToBase);
                    //iau wobble
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
