package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.XDrive;

public class AutonomousModeRedRight extends OpMode {
    private MotorMap driveMap;
    private XDrive xDrive;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false};

    private double currentYaw;
    private Robot robot;


    @Override
    public void init() {

        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobble", "clawLeft", "clawRight");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (!hasMoved[0]) {
            hasMoved[0] = robot.StrafeByDistance(1000, Math.PI/2, telemetry);
        } else if (!hasMoved[1]) {
            hasMoved[1] =  robot.Shoot(10);
        } else if (!hasMoved[2]) {
            hasMoved[2] = robot.RotateByAngleUsingIMU(Math.toRadians(15), true, telemetry);
        } else if (!hasMoved[3]) {
            try {
                hasMoved[3] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[4]) {
            hasMoved[4] = robot.StrafeByDistance(70, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[5]) {
            try {
                hasMoved[5] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[6]) {
            hasMoved[6] = robot.StrafeByDistance(70, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[7]) {
            try {
                hasMoved[7] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[8]) {
            hasMoved[8] = robot.StrafeByDistance(850, Math.PI/2-robot.GetYaw(), telemetry);
        } else {
            robot.SetStrafe(0,0);
            robot.SetRotation(0);
        }

    }

    @Override
    public void stop() {

    }
}
