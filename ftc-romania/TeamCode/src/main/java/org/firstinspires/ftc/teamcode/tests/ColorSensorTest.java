package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    ColorSensor color_sensor;

    @Override
    public void runOpMode(){
        color_sensor = hardwareMap.colorSensor.get("color_sensor_intake");
        color_sensor.enableLed(true);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("red", color_sensor.red());
            telemetry.addData("green", color_sensor.green());
            telemetry.addData("blue", color_sensor.blue());
            telemetry.addData("alpha", color_sensor.alpha());
            telemetry.addData("argb", color_sensor.argb());
        }
    }
}