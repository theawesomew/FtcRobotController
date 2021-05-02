package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;

public class Pushy extends Mechanisms {
    private Servo pushy;

    public Pushy(ServoMap servoMap) {
        pushy = servoMap.GetServoMap().get("pushServo");
    }

    public void SetPosition(double position) {
        pushy.setPosition(position);
    }

    public void Push () {
        pushy.setPosition(0.2);
        pushy.setPosition(0);
    }

}
