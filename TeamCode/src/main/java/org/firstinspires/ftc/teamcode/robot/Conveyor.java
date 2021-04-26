package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Conveyor extends Mechanisms {
    private DcMotor conveyor;

    public Conveyor(MotorMap mechMap) {
        conveyor = mechMap.GetMotorMap().get("conveyor");
    }

    public void SetPower(double power) {
        conveyor.setPower(power);
    }
}
