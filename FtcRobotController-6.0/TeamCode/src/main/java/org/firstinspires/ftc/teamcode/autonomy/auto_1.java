package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.servo_block;
import org.firstinspires.ftc.teamcode.hardware.servo_piston;

import java.util.List;

@Autonomous
public class auto_1 extends LinearOpMode {

    Hardware robot = new Hardware();
    double ok = 0.0;
    double lvl = 1.0;
    double speeeed = 0.7;// VITEZA CU CARE SE MISCA ROBOTUL  INTRE -1.0 SI 1.0
    double incet = 0.3;
    servo_piston servoPiston = new servo_piston();
    servo_block servoBlock = new servo_block();

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
    public void runOpMode() {

        robot.initHardware(hardwareMap);
        servoPiston.initPiston(hardwareMap);
        servoBlock.initBlock(hardwareMap);
        servoBlock.open();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            //tfod.setZoom(2.5, 1.78);
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

        while(opModeIsActive()) {
            telemetry.addData("Discuri", zona);

            if(ok == 0.0)
            {
                sleep(300);
                robot.totalPower(-speeeed,speeeed,speeeed,-speeeed,0,0,0);
                sleep(600 );          // TIMP STRAFE  1000 = 1 SEC
                robot.totalPower(0,0,0,0,0,0,0);
                sleep(800);

                ok = 0.5;
            }

            if(ok == 0.5){
                sleep(300);
                robot.totalPower(speeeed,speeeed,speeeed,speeeed,0,0,0);
                sleep(1100 );          // TIMP MERGE INAINTE  1000 = 1 SEC
                robot.totalPower(0,0,0,0,0,0,0);
                sleep(800);

                ok = 0.7;
            }

            if(ok == 0.7)
            {
                sleep(300);
                robot.totalPower(speeeed,-speeeed,-speeeed,speeeed,0,0,0);
                sleep(800 );          // TIMP STRAFE  1000 = 1 SEC
                robot.totalPower(0,0,0,0,0,0,0);
                sleep(800);

                ok = 1.0;
            }

            if(ok == 1.0){
                servoPiston.close();
                sleep(300);
                robot.totalPower(0,0,0,0,0, lvl,0);
                sleep(2000);

                ok = 2.0;
            }

            if(ok == 2.0){
                sleep(500);
                servoPiston.open();
                sleep(500);
                servoPiston.close();
                sleep(500);

                servoBlock.close();
                sleep(50);
                servoBlock.open();
                sleep(50);
                servoBlock.close();
                sleep(50);
                servoBlock.open();
                sleep(500);

                sleep(500);
                servoPiston.open();
                sleep(500);
                servoPiston.close();
                sleep(500);

                servoBlock.close();
                sleep(50);
                servoBlock.open();
                sleep(50);
                servoBlock.close();
                sleep(50);
                servoBlock.open();
                sleep(500);

                sleep(500);
                servoPiston.open();
                sleep(1000);
                servoPiston.close();

                robot.totalPower(0,0,0,0,lvl,0,0);
                sleep(500);

                ok = 3.5;
            }

            if(ok == 3.5)
            {
                if(zona == 0.0)
                {
                    ok = 4.0;
                }
                else
                {
                    ok = 3.7;
                }
            }

            if(ok == 3.7)
            {
                if(zona == 1)
                {
                    robot.totalPower(speeeed, -speeeed, speeeed, -speeeed, 0, 0,0);
                    sleep(1100);
                    robot.totalPower(0, 0, 0, 0, 0, 0,0);

                    servoBlock.close();
                    sleep(300);
                    robot.totalPower(speeeed,speeeed,speeeed,speeeed,lvl,0,0);
                    sleep(400);
                    robot.totalPower(0,0,0,0,lvl,0,0);
                    sleep(1000);
                    robot.totalPower(speeeed, -speeeed, speeeed, -speeeed, lvl, 0,0);
                    sleep(950);
                    robot.totalPower(0, 0, 0, 0, 0, 0,0);
                    sleep(200);
                    robot.totalPower(speeeed,speeeed,speeeed,speeeed,lvl,0,0);
                    sleep(300);
                    robot.totalPower(0,0,0,0,lvl,0,0);
                    sleep(500);
                    servoBlock.open();
                    sleep(50);
                    servoBlock.close();
                    sleep(50);
                    servoBlock.open();
                    sleep(200);
                    servoPiston.close();
                    sleep(300);
                    robot.totalPower(0,0,0,0,0, lvl,0);
                    sleep(2000);
                    sleep(500);
                    servoPiston.open();
                    sleep(500);
                    servoPiston.close();
                    sleep(500);
                    ok = 4.0;
                }
                else
                {
                    robot.totalPower(-speeeed, -speeeed, -speeeed, -speeeed, 0, 0,0);
                    sleep(400);
                    robot.totalPower(0, 0, 0, 0, 0, 0,0);

                    robot.totalPower(speeeed,speeeed,speeeed,speeeed,0,0,0);
                    sleep(500);
                    robot.totalPower(0,0,0,0,0,0,0);

                    robot.totalPower(speeeed, -speeeed, speeeed, -speeeed, 0, 0,0);
                    sleep(1250);
                    robot.totalPower(0, 0, 0, 0, 0, 0,0);

                    servoBlock.close();
                    robot.totalPower(incet,incet,incet,incet,lvl,0,0);
                    sleep(1650);
                    robot.totalPower(0,0,0,0,lvl,0,0);
                    sleep(300);

                    robot.totalPower(0,0,0,0,lvl,0,0);
                    sleep(1000);

                    robot.totalPower(-speeeed,-speeeed,-speeeed,-speeeed,lvl,0,0);
                    sleep(600);
                    robot.totalPower(0,0,0,0,0,0,0);


                    robot.totalPower(speeeed, -speeeed, speeeed, -speeeed, lvl, 0,0);
                    sleep(1200);
                    robot.totalPower(0, 0, 0, 0, lvl, 0,0);
                    sleep(500);

                    servoBlock.open();
                    sleep(50);
                    servoBlock.close();
                    sleep(50);
                    servoBlock.open();
                    sleep(200);

                    servoPiston.close();
                    sleep(300);
                    robot.totalPower(0,0,0,0,0, lvl,0);
                    sleep(1500);

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
                    sleep(500);


                    ok = 4.0;
                }
            }

            if(ok == 4.0){
                sleep(300);
                robot.totalPower(speeeed,speeeed,speeeed,speeeed,0,0,0);
                sleep(200);     // TIMP MERGE INAINTE DUPA LANSARE  1000 = 1 SEC
                robot.totalPower(0,0,0,0,0, 0.0,0);
                sleep(300);

                ok = 5.0;
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