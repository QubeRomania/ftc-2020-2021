package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_block {
    public Servo servo = null;

    public static double SERVO_RELEASE = 0.48;
    public static double SERVO_CLOSE = 0.35;

    public void initBlock(HardwareMap hwMap)
    {
        servo = hwMap.get(Servo.class, "servoBlock");
        close();
    }

    public void setServoPositions(double pos)
    {
        if(pos>-1.0)
        {
            servo.setPosition(pos);
        }
    }

    public void open() {
        setServoPositions(SERVO_RELEASE);
    }

    public void close()
    {
        setServoPositions(SERVO_CLOSE);
    }
}
