package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_wobble {
    public Servo servo = null;

    public static double SERVO_RELEASE = 0.4; //0.12/lk
    public static double SERVO_CLOSE = 0; //0.6

    public void initWobble(HardwareMap hwMap, Boolean value)
    {
        servo = hwMap.get(Servo.class, "servoWobble");
        if(value == Boolean.TRUE)
            close();
        else
            open();
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
