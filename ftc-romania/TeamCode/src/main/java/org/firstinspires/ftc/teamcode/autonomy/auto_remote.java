package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_perete;
import org.firstinspires.ftc.teamcode.hardware.servo_wobble;


import android.graphics.ColorSpace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
public class auto_remote extends LinearOpMode
{
    public SampleMecanumDrive bot;
    public double towerPower = 1620;
    public double powershotPower = 1450;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(52, 0, 4.6, 14);

    public static double zero = 128;
    public static double unu = 136;
    public static double patru = 141;

    Boolean cont = Boolean.FALSE;
    Boolean contor = Boolean.FALSE;

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;

    servo_block servoBlock = new servo_block();
    servo_wobble servoWobble = new servo_wobble();
    servo_perete servoPerete = new servo_perete();

    @Override
    public void runOpMode()
    {

        Gamepad gp1 = gamepad1;

        bot = new SampleMecanumDrive(hardwareMap);

        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        servoWobble.initWobble(hardwareMap, true);
        servoPerete.initPerete(hardwareMap);
        bot.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);


        // *****************************  ZERO RINGS  ***************************** \\

        Trajectory trajShoot = bot.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(powershotPower);
                })
                .splineTo(new Vector2d(63.5, 7), Math.toRadians(-1))
                .build();

        Trajectory putAwayWobble10 = bot.trajectoryBuilder(trajShoot.end(),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToLinearHeading(new Pose2d(82, -35, Math.toRadians(80)))
                .build();

        Trajectory returnBack0 = bot.trajectoryBuilder(putAwayWobble10.end())
                .splineToLinearHeading(new Pose2d(34,-18,Math.toRadians(30)),0)
                .addTemporalMarker(1,()->{
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        Trajectory takeWobble20 = bot.trajectoryBuilder(returnBack0.end())
                .lineTo(new Vector2d(27,-25))
                .build();

        Trajectory putAwayWobble20 = bot.trajectoryBuilder(takeWobble20.end())
                .lineToLinearHeading(new Pose2d(80,-28, Math.toRadians(60)))
                .build();

        Trajectory goForward0 = bot.trajectoryBuilder(putAwayWobble20.end())
                .splineTo(new Vector2d(117,-30), Math.toRadians(90))
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory takeRings0 = bot.trajectoryBuilder(goForward0.end())
                .addTemporalMarker(2,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .strafeTo(new Vector2d(118,10))
                .splineTo(new Vector2d(60,-10),Math.toRadians(-10))
                .build();

        Trajectory park0 = bot.trajectoryBuilder(takeRings0.end())
                .lineTo(new Vector2d(70,0))
                .build();
        waitForStart();




        // *****************************  ONE RING  ***************************** \\


        Trajectory putAwayWobble11 = bot.trajectoryBuilder(trajShoot.end(),true)
                .addTemporalMarker(0, () -> {
                    bot.outtakeMotor.setVelocity(0);
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineToLinearHeading(new Pose2d(103, -9, Math.toRadians(90)))
                .build();

        Trajectory returnBack1 = bot.trajectoryBuilder(putAwayWobble10.end())
                .splineTo(new Vector2d(108,15),Math.toRadians(180))
                .splineTo(new Vector2d(80,0), Math.toRadians(180))
                .splineTo(new Vector2d(45,-20), Math.toRadians(180))
                .addTemporalMarker(1,()->{
                    bot.wobbleMotor.setTargetPosition(900);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addTemporalMarker(0, ()->{
                    bot.intakeMotor.setPower(1);
                    servoBlock.close();
                })
                .build();

        Trajectory goBack1 = bot.trajectoryBuilder(returnBack1.end())
                .splineToLinearHeading(new Pose2d(34,-18,Math.toRadians(30)),0)
                .build();

        Trajectory takeWobble21 = bot.trajectoryBuilder(goBack1.end())
                .lineTo(new Vector2d(27,-25))
                .build();

        Trajectory trajShooting1 = bot.trajectoryBuilder(takeWobble21.end())
                .addTemporalMarker(0, ()->{
                    servoBlock.open();
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .lineToLinearHeading(new Pose2d(63,-20,Math.toRadians(-2)))
                .build();

        Trajectory putAwayWobble21 = bot.trajectoryBuilder(trajShooting1.end())
                .lineToLinearHeading(new Pose2d(79,-25, Math.toRadians(180)))
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(0);
                })
                .build();

        Trajectory park1 = bot.trajectoryBuilder(putAwayWobble21.end())
                .lineTo(new Vector2d(70,-25))
                .build();

        // *****************************  FOUR RINGS  ***************************** \\

        Trajectory goToRings4 = bot.trajectoryBuilder(trajShoot.end(), true)
                .splineToConstantHeading(new Vector2d(22, -18),0)
                .build();

        Trajectory take1Ring4 = bot.trajectoryBuilder(goToRings4.end())
                .strafeTo(new Vector2d(34,-21),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(23, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0,()->{
                    bot.intakeMotor.setPower(1);
                })
                .build();

        Trajectory getBack4 = bot.trajectoryBuilder(take1Ring4.end())
                .lineToLinearHeading(new Pose2d(30,-20, Math.toRadians(-4)))
                .addTemporalMarker(0,()->{
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .build();

        Trajectory goShoot4 = bot.trajectoryBuilder(getBack4.end())
                .strafeTo(new Vector2d(53,-20))
                .splineTo(new Vector2d(63.5,-16),Math.toRadians(-3))
                .addTemporalMarker(0,()->{
                    servoBlock.close();
                    bot.intakeMotor.setPower(1);
                    bot.outtakeMotor.setVelocity(towerPower);
                })
                .build();

        Trajectory putAwayWobble14 = bot.trajectoryBuilder(goShoot4.end())
                .splineToLinearHeading(new Pose2d(115,-30, Math.toRadians(90)),0)
                .addTemporalMarker(0,()->{
                    bot.wobbleMotor.setTargetPosition(950);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        Trajectory park4 = bot.trajectoryBuilder(putAwayWobble14.end())
                .lineToLinearHeading(new Pose2d(70,-30,Math.toRadians(0)))
                .addTemporalMarker(0, ()->{
                    bot.wobbleMotor.setTargetPosition(0);
                    bot.wobbleMotor.setPower(0.2);
                    bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .build();

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        //phoneCam.setPipeline(pipeline);
        webcam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        while(!isStarted())
        {
            telemetry.addData("Analysis", pipeline.avg1);
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("press x for 0 -", zero);
            telemetry.addData("press y for 1 -", unu);
            telemetry.addData("press a for 4 -", patru);
            telemetry.update();

            /*if(gp1.x)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 0 -", zero);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        zero = zero + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        zero = zero - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }

            if(gp1.y)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 1 -", unu);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        unu = unu + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        unu = unu - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }

            if(gp1.a)
            {
                while(gp1.b == false)
                {
                    telemetry.addData("value for 4 -", patru);
                    telemetry.addData("press dpad up to increase by", 1);
                    telemetry.addData("press dpad down to decrease by", 1);
                    telemetry.addData("press b to save and exit", "");
                    telemetry.update();

                    if (gp1.dpad_up && !cont){
                        patru = patru + 1;
                        cont = !cont;
                    }
                    if (gp1.dpad_down && !contor){
                        patru = patru - 1;
                        contor = !contor;
                    }
                    if (!gp1.dpad_up){
                        cont = false;
                    }
                    if (!gp1.dpad_down){
                        contor = false;
                    }
                }
            }
             */
        }

        waitForStart();


        while (opModeIsActive())
        {

            if(pipeline.zona == 0)
            {
                bot.followTrajectory(trajShoot);
                shoot(1, false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.outtakeMotor.setVelocity(0);
                bot.followTrajectory(putAwayWobble10);
                servoWobble.open();
                sleep(200);
                bot.wobbleMotor.setTargetPosition(300);
                bot.wobbleMotor.setPower(0.2);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                bot.followTrajectory(returnBack0);
                bot.followTrajectory(takeWobble20);
                servoWobble.close();
                sleep(300);
                bot.followTrajectory(putAwayWobble20);
                servoWobble.open();
                sleep(100);
                bot.wobbleMotor.setTargetPosition(0);
                bot.wobbleMotor.setPower(0.2);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                bot.followTrajectory(goForward0);
                bot.followTrajectory(takeRings0);
                servoBlock.open();
                sleep(600);
                shoot(3,false);
                bot.followTrajectory(park0);
            }

            else if(pipeline.zona == 1)
            {
                bot.followTrajectory(trajShoot);
                shoot(1, false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.outtakeMotor.setVelocity(0);
                bot.followTrajectory(putAwayWobble11);
                servoWobble.open();
                sleep(200);
                bot.wobbleMotor.setTargetPosition(300);
                bot.wobbleMotor.setPower(0.2);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                bot.followTrajectory(returnBack1);
                bot.followTrajectory(goBack1);
                bot.followTrajectory(takeWobble21);
                servoWobble.close();
                sleep(300);
                bot.followTrajectory(trajShooting1);
                bot.intakeMotor.setPower(0);
                sleep(200);
                shoot(3,false);
                bot.followTrajectory(putAwayWobble21);
                servoWobble.open();
                sleep(100);
                bot.wobbleMotor.setTargetPosition(0);
                bot.wobbleMotor.setPower(0.2);
                bot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(300);
                bot.followTrajectory(park1);
            }

            else if(pipeline.zona == 4)
            {
                bot.followTrajectory(trajShoot);
                shoot(1, false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.turn(Math.toRadians(-6));
                shoot(1,false);
                bot.outtakeMotor.setVelocity(0);
                bot.followTrajectory(goToRings4);
                servoBlock.close();
                bot.followTrajectory(take1Ring4);
                bot.followTrajectory(getBack4);
                sleep(700);
                bot.intakeMotor.setPower(0);
                servoBlock.setServoPositions(0.68);
                sleep(400);
                shoot(2,false);
                bot.followTrajectory(goShoot4);
                sleep(1700);
                bot.intakeMotor.setPower(0);
                servoBlock.open();
                sleep(400);
                shoot(3,false);
                bot.outtakeMotor.setVelocity(0);
                bot.followTrajectory(putAwayWobble14);
                servoWobble.open();
                sleep(200);
                bot.followTrajectory(park4);
            }
            stop();

        }


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

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        public double zona = 1;

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(260,40);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 30;

        final int FOUR_RING_THRESHOLD = (int) Math.round((patru+unu)/2);
        final int ONE_RING_THRESHOLD = (int) Math.round((zero+unu)/2);

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -5); // Thickness of the rectangle lines

            position = RingPosition.ONE; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
                zona = 4;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                zona = 1;
            }else{
                position = RingPosition.NONE;
                zona = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -5); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}