package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooter extends Mechanisms {
    private DcMotor shooter;

    public Shooter(MotorMap mechMap) {
        shooter = mechMap.GetMotorMap().get("shoot");
    }

    public void SetPower(double power) {
        shooter.setPower(power);
    }

}
