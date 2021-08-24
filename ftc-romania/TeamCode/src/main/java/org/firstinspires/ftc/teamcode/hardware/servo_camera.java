package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_camera {
    public Servo servo = null;

    public static double SERVO_RELEASE = 1;
    public static double SERVO_RED_LEFT = 0.75;
    public static double SERVO_RED_RIGHT = 0.56;
    public static double SERVO_BLUE_LEFT = 0.76;
    public static double SERVO_BLUE_RIGHT = 0.56;

    public void initCamera(HardwareMap hwMap)
    {
        servo = hwMap.get(Servo.class, "servoCamera");
    }

    public void setServoPositions(double pos)
    {
        if(pos>-1.0)
        {
            servo.setPosition(pos);
        }
    }

    public void redleft(){setServoPositions(SERVO_RED_LEFT);}

    public void redright(){setServoPositions(SERVO_RED_RIGHT);}

    public void blueleft(){setServoPositions(SERVO_BLUE_LEFT);}

    public void blueright(){setServoPositions(SERVO_BLUE_RIGHT);}

    public void open() {
        setServoPositions(SERVO_RELEASE);
    }
}
