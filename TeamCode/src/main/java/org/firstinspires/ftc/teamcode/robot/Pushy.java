package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pushy extends Mechanisms {
    private Servo pushy;

    public Pushy (HardwareMap hardwareMap, String servoName) {
        pushy = hardwareMap.servo.get(servoName);
        pushy.setPosition(1);

    }

    public void Push () {
        pushy.setPosition(0);

    }

    public void Retract () {
        pushy.setPosition(1);
    }

    public boolean PushThenRetract (Telemetry telemetry) throws InterruptedException {
        this.Push();
        Thread.sleep(500);
        this.Retract();
        return true;
    }
}
