package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends Mechanisms{

    private Servo clawLeft;
    private Servo clawRight;

    public Claw (HardwareMap hardwareMap, String servoLeft, String ServoRight) {
        clawLeft = hardwareMap.servo.get(servoLeft);
        clawRight = hardwareMap.servo.get(ServoRight);

        clawLeft.setPosition(0);
        clawRight.setPosition(1);
    }


    public void ClawOpen() {
        clawRight.setPosition(1);
        clawLeft.setPosition(0);
    }

    public void ClawClose() {
        clawRight.setPosition(0);
        clawLeft.setPosition(1);
    }


}
