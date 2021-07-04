package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pushy extends Mechanisms {
    private Servo pushy;
    private boolean started = false;
    private ElapsedTime timer;

    public Pushy (HardwareMap hardwareMap, String servoName) {
        pushy = hardwareMap.servo.get(servoName);
        pushy.setPosition(1);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void Push () {
        pushy.setPosition(0);

    }

    public void Retract () {
        pushy.setPosition(1);
    }

    public boolean PushThenRetract (Telemetry telemetry) {
        if (!started) {
            timer.reset();
            started = true;
        }

        if (timer.milliseconds() <= 300) {
            this.Push();
        } else if (timer.milliseconds() > 300 && timer.milliseconds() <= 600) {
            this.Retract();
        } else {
            started = false;
            return true;
        }
        return false;
    }
}
