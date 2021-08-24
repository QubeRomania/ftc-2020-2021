package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;
import org.firstinspires.ftc.teamcode.hardware.servo_camera;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;
import org.firstinspires.ftc.teamcode.teleopGame.PoseStorage;

import java.util.List;

public abstract class AutoBase extends LinearOpMode{

    public SampleMecanumDrive bot;
    public double powershotPower = 1250;
    public double towerPower = 1370;
    public double blocPos = 0.17;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(70, 0, 15, 14);

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //developer key
    private static final String VUFORIA_KEY =
            "AfEZkiL/////AAABmaamsSiv9Ekeqgkncg9w6UKHCKU6c+yfO47f26KKNNRR5bu3Dqtk1794PqGUq3NuWygJPWjhsUkSBbDXsvzWfpHhGvZPx+TII7Io4o7hB8uAul0lxS1eywdu1374gI74XkwUD3rp08eW1EGW8nSrMXrwQgpx4ivWP7eVAFRGtciEYZ6+XC/tK0R9csWhRalAw1fcgFHJDFeO6NK3xgk01vVAItO3GRXjzEim9um6iWC70s67xCRFM+s2j+0oVCVyop5aPZ71Sn7k6wcSGW+eAgVtfNRslSBhEkUXaH0ThS6QaCeNhC4F44kuMpwMlrIwxZiiHJX1muex8zfcGN8alM+b67q8sJ8kO+QY5ePgMkxQ";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public double zona = 0;

    //================================================ RED ZONE ===========================================
    public Vector2d highVectorRed = new Vector2d(63,0);
    public double highAngleRed = Math.toRadians(9);

    public Vector2d powershotVectorRed = new Vector2d(63,0);
    public double powershotAngleRed = Math.toRadians(2);

    //================================================ BLUE ZONE ==========================================
    public Vector2d highVectorBlue = new Vector2d(63,-5);
    public double highAngleBlue = Math.toRadians(8);

    public Vector2d powershotVectorBlue = new Vector2d(63,-8.5);
    public double powershotAngleBlue = Math.toRadians(0);

    public Pose2d fin = new Pose2d(0,0,0);

    public servo_block servoBlock = new servo_block();
    public servo_wobble servoWobble = new servo_wobble();
    public servo_perete servoPerete = new servo_perete();
    public servo_camera servoCamera = new servo_camera();
    public servo_piston servoPiston = new servo_piston();

    public void initRobot()
    {
        bot = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        servoCamera.initCamera(hardwareMap);
        servoPiston.initPiston(hardwareMap);
        bot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        initCamera();
    }

    public void initCamera() {
        initVuforia();
        initTfod();
        if(tfod != null)
        {
            tfod.activate();
            tfod.setZoom(4.0, 16.0/9.0);
        }
    }

    public void recognizeRings()
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

    public void shootPowershotsRed()
    {
        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(powershotPower);
                })
                .splineTo(powershotVectorRed, powershotAngleRed)
                .build();
        fin = trajShoot.end();
        bot.followTrajectory(trajShoot);
        servoPerete.open();
        sleep(2000);
        shoot(1);
        bot.turn(Math.toRadians(-6));
        shoot(1);
        bot.turn(Math.toRadians(-6));
        shoot(1);
        bot.outtakeMotor.setVelocity(0);
    }

    public void shootPowershotsBlue()
    {
        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(powershotPower+25);
                })
                .splineTo(powershotVectorBlue, powershotAngleBlue)
                .build();
        fin = trajShoot.end();
        bot.followTrajectory(trajShoot);
        servoPerete.open();
        sleep(200);
        shoot(1);
        bot.turn(Math.toRadians(-6));
        shoot(1);
        bot.outtakeMotor.setVelocity(powershotPower+20);
        sleep(100);
        bot.turn(Math.toRadians(-6));
        shoot(1);
        bot.outtakeMotor.setVelocity(0);
    }

    public void shoothighGoalRed()
    {
        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .splineTo(highVectorRed,highAngleRed)
                .build();
        fin = trajShoot.end();
        bot.followTrajectory(trajShoot);
        sleep(200);
        shoot(3);

    }

    public void shoothighGoalBlue()
    {
        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, ()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .splineTo(highVectorBlue,highAngleBlue)
                .build();
        fin = trajShoot.end();
        bot.followTrajectory(trajShoot);
        shoot(5);
    }

    public void shoot(int rings)
    {
        for(int i=1;i<=rings;++i)
        {
            movePiston(750);
        }
    }

    public void cameraPosition(String name)
    {
        if(name == "redleft")
            servoCamera.redleft();
        if(name == "redright")
            servoCamera.redright();
        if(name == "blueleft")
            servoCamera.blueleft();
        if(name == "blueright")
            servoCamera.blueright();
    }

    public void releaseWobble()
    {
        servoWobble.open();
        sleep(200);
    }

    public void movePiston(int timeToSleep)
    {
        servoPiston.open();
        sleep(100);
        servoPiston.close();
        sleep(timeToSleep);
    }

    public void moveWobble(int position, double power, int timeToSleep)
    {
        bot.wobbleMotor.setPower(power);
        bot.wobbleMotor.setTargetPosition(position);
        bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(timeToSleep);
    }
}
