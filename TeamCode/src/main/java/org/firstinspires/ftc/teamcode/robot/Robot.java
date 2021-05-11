package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private XDrive xDrive;
    private Conveyor conveyor;
    private Intake intake;
    private Pushy pushy;
    private Shooter shooter;

    public Robot (HardwareMap hardwareMap, MotorMap driveMap, String conveyorName, String pushyName,
                  String intakeName, String shooterName) {
        this.xDrive = new XDrive(driveMap);
        this.conveyor = new Conveyor(hardwareMap, conveyorName);
        this.intake = new Intake(hardwareMap, intakeName);
        this.pushy = new Pushy(hardwareMap, pushyName);
        this.shooter = new Shooter(hardwareMap, shooterName);
    }

    public void Drive (Telemetry telemetry) {
        this.xDrive.Drive(telemetry);
    }

    public void SetStrafe (double power, double angle) {
        this.xDrive.SetStrafe(power, angle);
    }

    public boolean StrafeByDistance (double distance, double angle, Telemetry telemetry) {
        return this.xDrive.StrafeByDistance(distance, angle, telemetry);
    }

    public void SetRotation (double power) {
        this.xDrive.SetRotation(power);
    }

    public void RotateByAngle (double angle, boolean direction, Telemetry telemetry) {
        this.xDrive.RotateByAngle(angle, direction, telemetry);
    }

    public void SetConveyorPower (double power) {
        this.conveyor.SetPower(-power);
    }

    public boolean RunConveyor () {
        return this.conveyor.runConveyor();
    }

    public void SetShooterPower (double power) {
        this.shooter.SetPower(power);
    }

    public boolean Shoot () {
        return this.shooter.Shoot();
    }

    public void SetIntakePower (double power) {
        this.intake.SetPower(power);
    }

    public void Push () {
        this.pushy.Push();
    }

    public void Retract () {
        this.pushy.Retract();
    }
}
