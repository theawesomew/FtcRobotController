package org.firstinspires.ftc.teamcode.oldAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="AutonomousModeRedLeft", group="Autonomous")
public class AutonomousModeRedLeft extends OpMode {
    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false, false};
    private double currentYaw;


    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight", "clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4", "wobbleMotor");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Yaw", robot.GetYaw());
        telemetry.addData("Roll", robot.GetRoll());
        telemetry.addData("Pitch", robot.GetPitch());
        telemetry.addData("Voltage:", voltageSensor.getVoltage());

        if (!hasMoved[0]) {
            hasMoved[0] = robot.StrafeByDistance(950, Math.PI/2, telemetry);
        } else if (!hasMoved[1]) {
            hasMoved[1] = robot.StrafeByDistance(700, Math.PI, telemetry);
        } else if (!hasMoved[2]) {
            hasMoved[2] = robot.Shoot(10);
        } else if (!hasMoved[3]) {
            hasMoved[3] = robot.RotateByAngleUsingIMU(Math.toRadians(15), true, telemetry);
        } else if (!hasMoved[4]) {
            hasMoved[4] = robot.PushThenRetract(telemetry);
        } else if (!hasMoved[5]) {
            hasMoved[5] = robot.StrafeByDistance(70, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[6]) {
            hasMoved[6] = robot.PushThenRetract(telemetry);
        } else if (!hasMoved[7]) {
            hasMoved[7] = robot.StrafeByDistance(80, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[11]) {
            hasMoved[11] = robot.RotateByAngleUsingIMU(Math.toRadians(1), true, telemetry);
        } else if (!hasMoved[8]) {
            hasMoved[8] = robot.PushThenRetract(telemetry);
        } else if (!hasMoved[9]) {
            hasMoved[9] = robot.StrafeByDistance(1100, Math.PI/2-robot.GetYaw(), telemetry);
        } else if (!hasMoved[10]) {
            hasMoved[10] = true;
        } else {
            robot.SetStrafe(0,0);
            robot.SetRotation(0);
        }
    }

    @Override
    public void stop() {

    }
}
