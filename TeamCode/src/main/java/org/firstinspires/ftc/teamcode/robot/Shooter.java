package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter extends Mechanisms {
    private DcMotor shooter;

    public Shooter(HardwareMap hardwareMap, String shooterName) {
        shooter = hardwareMap.dcMotor.get(shooterName);
    }

    public void SetPower(double power) {
        shooter.setPower(power);
    }

    public boolean Shoot (double rotations) {
        if (rotations * 1440 != Math.round(rotations * 1440)) {
            rotations = Math.round(rotations * 1440) / 1440;
        }

        if (!shooter.isBusy()) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setTargetPosition((int) rotations * 1440);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.SetPower(1);
        }
        return true;
    }


}
