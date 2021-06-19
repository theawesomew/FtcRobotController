package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ramp extends Mechanisms {
    private Servo ramp;


    public Ramp (HardwareMap hardwareMap, String servoName) {
        ramp = hardwareMap.servo.get(servoName);
        ramp.setPosition(0.5);
    }

    public void Retract() {
        ramp.setPosition(0.9);
    }

    public void Extend() {ramp.setPosition(0.15);}

    public boolean Up() {ramp.setPosition(0.15); return true;}

    public boolean Down() {ramp.setPosition(0.9); return true;}

    public void Latch() {ramp.setPosition(0.6);}

}
