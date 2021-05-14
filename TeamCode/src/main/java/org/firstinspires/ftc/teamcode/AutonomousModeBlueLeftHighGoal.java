package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobble");
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

        if (!hasMoved[0]) {
            hasMoved[0] = robot.StrafeByDistance(1300, Math.PI/2, telemetry);
        } else if (!hasMoved[1]) {
            hasMoved[1] = robot.Shoot(7.5);
        } else if (!hasMoved[2]) {
            hasMoved[2] = robot.RotateByAngleUsingIMU(Math.toRadians(5), true, telemetry);
        } else if (!hasMoved[3]) {
            try {
                hasMoved[3] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[4]) {
            try {
                hasMoved[4] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[5]) {
            try {
                hasMoved[5] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[6]) {
            hasMoved[6] = robot.StrafeByDistance(200, Math.PI/2 - Math.toRadians(5), telemetry);
        } else {
            robot.SetStrafe(0,0);
            robot.SetRotation(0);
        }




    }

    @Override
    public void stop() {

    }



}
