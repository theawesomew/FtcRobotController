package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="TestOpMode", group="Autonomous")
public class TestOpMode extends OpMode {
    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false};

    private double currentYaw;

    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft" , "wobbleRight", "clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4");
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

        if (!hasMoved[0]) {
            hasMoved[0] = robot.RotateByAngleUsingIMU(Math.PI/4, true, telemetry);
        } else if (!hasMoved[1]) {
            telemetry.addData("Robot Yaw", robot.GetYaw());
            hasMoved[1] = robot.StrafeByDistance(1000, Math.PI/2 - robot.GetYaw(), telemetry);
        } else {
            robot.SetStrafe(0,0);
            robot.SetRotation(0);
        }
    }

    @Override
    public void stop() {

    }

}
