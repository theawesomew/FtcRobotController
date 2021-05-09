package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pushy extends Mechanisms {
    private Servo pushy;

    public Pushy (HardwareMap hardwareMap, String servoName) {
        pushy = hardwareMap.servo.get(servoName);
        pushy.setPosition(0.2);
    }

    public void Push () {
        pushy.setPosition(0);
    }

    public void Retract () {
        pushy.setPosition(0.2);
    }
}
