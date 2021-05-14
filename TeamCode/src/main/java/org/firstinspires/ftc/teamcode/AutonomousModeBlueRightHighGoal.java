package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Pushy;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoMap;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.XDrive;

@Autonomous(name="UltimateGoalAutonomousModeBlueRightHighGoal", group="Autonomous")
public class AutonomousModeBlueRightHighGoal extends OpMode {
    private static final String vuforiaKey = "AV98RL7/////AAABmYXDdoEmIUn1v6Ppa4Z/oN0tDQt5nJk+KD9Gy+XiJe1DpevozXumH++UVyRGG8Al6PX2as4EddLKpGncqMsiDQeugSuOXBAKBVnpGda6+GX6veXRgYkOEwq4HDxSPi3Nfqoe8/6GVo0TH5sqyOfCgZLIk2o2rzjmrrCbcou31JRGpg25elDXgbtXQcD+qPq748IrnJLh7/vbsk9tBANafFczL8l2mesx8Rj8i00T3x9JIHqPku9j3cUReAzTxa6X7vq/5IC2AtS05lFjmjlNkJRgnxVAwBjAgFtYBH2O8eXGUtY147+ABdxJLpmIbeOZDvZ38k8NByzEV2RfQCSDYhbYBOKwlpGqn7hX9xyHesAs";
    private static final String tfodModelAsset = "UltimateGoal.tflite";
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tfod;

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

        /*
        forward
        start shooting
        rotate
        push ring
        move right
        push ring
        move right
        push ring
        move to line
        stop
         */


        if (!hasMoved[0]) {
            hasMoved[0] = robot.StrafeByDistance(1000, Math.PI/2, telemetry);
        } else if (!hasMoved[1]) {
            hasMoved[1] = robot.Shoot(10);
        } else if (!hasMoved[2]) {
            hasMoved[2] = robot.RotateByAngleUsingIMU(Math.toRadians(7), true, telemetry);
        } else if (!hasMoved[3]) {
            try {
                hasMoved[3] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[4]) {
            hasMoved[4] = robot.StrafeByDistance(5, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[5]) {
            try {
                hasMoved[5] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[6]) {
            hasMoved[6] = robot.StrafeByDistance(5, Math.PI-robot.GetYaw(), telemetry);
        } else if (!hasMoved[7]) {
            try {
                hasMoved[7] = robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!hasMoved[8]) {
            hasMoved[8] = robot.StrafeByDistance(900, Math.PI/2-robot.GetYaw(), telemetry);
        } else {
            robot.SetStrafe(0,0);
            robot.SetRotation(0);
        }

    }

    @Override
    public void stop() {

    }

    private void InitVuforia () {
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = vuforiaKey;
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(vuforiaParameters);
    }

    private void InitTFOD () {
        int tfodMonitorViewID = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewID);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocalizer);
        tfod.loadModelFromAsset(tfodModelAsset, "Single", "Quad");
    }
}
