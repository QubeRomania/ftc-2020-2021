package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_piston {
    public Servo servo = null;

    public static double SERVO_RELEASE = 0.18;
    public static double SERVO_CLOSE = 0.0;

    public void initPiston(HardwareMap hwMap)
    {
        servo = hwMap.get(Servo.class, "servoPiston");
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
