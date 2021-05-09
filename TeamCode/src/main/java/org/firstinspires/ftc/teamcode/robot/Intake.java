package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends Mechanisms {
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap, String intakeName) {
        intake = hardwareMap.dcMotor.get(intakeName);
    }

    public void SetPower(double power) {
        intake.setPower(power);
    }
}
