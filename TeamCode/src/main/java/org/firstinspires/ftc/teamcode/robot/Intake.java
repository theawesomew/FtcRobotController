package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake extends Mechanisms {
    private DcMotor intake;

    public Intake(MotorMap mechMap) {
        intake = mechMap.GetMotorMap().get("intake");
    }

    public void SetPower(double power) {
        intake.setPower(power);
    }
}
