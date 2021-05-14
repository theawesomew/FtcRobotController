package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter extends Mechanisms {
    private DcMotor shooter;
    private VoltageSensor voltageSensor;
    private double minSpeed = 0.97, maxSpeed = 1;

    private double c = 84 * (maxSpeed-minSpeed);
    private double a = 7 * minSpeed - 6 * maxSpeed;

    public Shooter(HardwareMap hardwareMap, String shooterName) {
        shooter = hardwareMap.dcMotor.get(shooterName);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
            this.SetPower((c/voltageSensor.getVoltage()) + a);
        }
        return true;
    }


}
