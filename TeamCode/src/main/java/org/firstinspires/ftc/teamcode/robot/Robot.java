package org.firstinspires.ftc.teamcode.robot;

public class Robot {
    private XDrive xDrive;
    private Conveyor conveyor;
    private Intake intake;
    private Pushy pushy;
    private Shooter shooter;

    public Robot (MotorMap driveMech) {
        xDrive = new XDrive(driveMech);

    }
}
