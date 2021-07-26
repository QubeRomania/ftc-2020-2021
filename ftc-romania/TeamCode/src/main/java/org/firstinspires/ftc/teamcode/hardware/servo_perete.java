package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_perete {
    public Servo servo = null;

    public static double SERVO_RELEASE = 0.69;
    public static double SERVO_CLOSE = 0.04;

    public void initPerete(HardwareMap hwMap)
    {
        servo = hwMap.get(Servo.class, "servoPerete");
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
