package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoalServo extends Mechanisms {

    private Servo wobbleServo;

    public WobbleGoalServo (HardwareMap hardwareMap, String wobbleServoName) {
        wobbleServo = hardwareMap.servo.get(wobbleServoName);
        wobbleServo.setPosition(0);
    }

    public void active () {
        wobbleServo.setPosition(0.5);
    }

    public void inactive () {
        wobbleServo.setPosition(0);
    }
}
