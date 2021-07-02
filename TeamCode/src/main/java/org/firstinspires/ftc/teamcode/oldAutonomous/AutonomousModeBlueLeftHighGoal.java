package org.firstinspires.ftc.teamcode.oldAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="UltimateGoalAutonomousModeBlueLeftHighGoal", group="Autonomous")

public class AutonomousModeBlueLeftHighGoal extends OpMode {

    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false};
    private ElapsedTime timer = new ElapsedTime();

    private double currentYaw;

    @Override
    public void init() {
        timer.reset();

        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake",
                "shooter", "wobbleLeft", "wobbleRight",
                "clawLeft", "clawRight", "ramp",
                "colorSensorRight1", "colorSensorRight4",
                "colorSensorLeft1", "colorSensorLeft4",
                "wobbleMotor", "wobbleGoalServo");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Voltage:", voltageSensor.getVoltage());
        telemetry.addData("Yaw", robot.GetYaw());
        if (timer.milliseconds() > 5000) {
            if (!hasMoved[0]) {
                hasMoved[0] = robot.StrafeByDistance(1010, Math.PI / 2, telemetry);
            } else if (!hasMoved[1]) {
                hasMoved[1] = robot.Shoot(7.5);
            } else if (!hasMoved[2]) {
                hasMoved[2] = robot.RotateByAngleUsingIMU(Math.toRadians(20), true, telemetry);
            } else if (!hasMoved[3]) {
                hasMoved[3] = robot.PushThenRetract(telemetry);
            } else if (!hasMoved[4]) {
                hasMoved[4] = robot.StrafeByDistance(5, Math.PI / 2 - robot.GetYaw(), telemetry);
            } else if (!hasMoved[5]) {
                hasMoved[5] = robot.PushThenRetract(telemetry);
            } else if (!hasMoved[6]) {
                hasMoved[6] = robot.StrafeByDistance(5, Math.PI / 2 - robot.GetYaw(), telemetry);
            } else if (!hasMoved[7]) {
                hasMoved[7] = robot.PushThenRetract(telemetry);
            } else if (!hasMoved[8]) {
                hasMoved[8] = robot.StrafeByDistance(700, Math.PI / 2 - robot.GetYaw(), telemetry);
            } else {
                robot.SetStrafe(0, 0);
                robot.SetRotation(0);
            }
        }




    }

    @Override
    public void stop() {

    }



}
