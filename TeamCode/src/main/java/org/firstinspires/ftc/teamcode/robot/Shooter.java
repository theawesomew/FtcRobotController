package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter extends Mechanisms {
    private DcMotor shooter;
    private VoltageSensor voltageSensor;
    private double minSpeed = 0.95, maxSpeed = 1;

    private double c = 60 * (maxSpeed-minSpeed);
    private double a = 5 * minSpeed - 4 * maxSpeed;

    public Shooter(HardwareMap hardwareMap, String shooterName) {
        shooter = hardwareMap.dcMotor.get(shooterName);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double GetPower () {
        return shooter.getPower();
    }

    public void SetPower(double power) {
        shooter.setPower(power);
    }

    public void AdjustedShootPower () {
        shooter.setPower(c/voltageSensor.getVoltage()+a);
    }

    public boolean Shoot (double rotations) {
        if (rotations * 28 != Math.round(rotations * 28)) {
            rotations = Math.round(rotations * 28) / 28;
        }

        if (!shooter.isBusy()) {
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setTargetPosition((int) rotations * 28 );
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.AdjustedShootPower();
        }
        return true;
    }





}
