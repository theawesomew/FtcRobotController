package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Mechanisms {

    private Servo wobbleServo;

    public WobbleArm (HardwareMap hardwareMap, String mechanismName) {
        wobbleServo = hardwareMap.servo.get(mechanismName);
        wobbleServo.setPosition(0);
    }

    public void Raise() {
        wobbleServo.setPosition(1);
    }

    public void Lower() {
        wobbleServo.setPosition(0);
    }
}
