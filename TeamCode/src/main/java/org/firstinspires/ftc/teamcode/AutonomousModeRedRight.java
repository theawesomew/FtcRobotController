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
import org.firstinspires.ftc.teamcode.robot.ServoMap;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.XDrive;

@Autonomous(name="UltimateGoalAutonomousModeRedRight", group="Autonomous")
public class AutonomousModeRedRight extends OpMode {
    private static final String vuforiaKey = "AV98RL7/////AAABmYXDdoEmIUn1v6Ppa4Z/oN0tDQt5nJk+KD9Gy+XiJe1DpevozXumH++UVyRGG8Al6PX2as4EddLKpGncqMsiDQeugSuOXBAKBVnpGda6+GX6veXRgYkOEwq4HDxSPi3Nfqoe8/6GVo0TH5sqyOfCgZLIk2o2rzjmrrCbcou31JRGpg25elDXgbtXQcD+qPq748IrnJLh7/vbsk9tBANafFczL8l2mesx8Rj8i00T3x9JIHqPku9j3cUReAzTxa6X7vq/5IC2AtS05lFjmjlNkJRgnxVAwBjAgFtYBH2O8eXGUtY147+ABdxJLpmIbeOZDvZ38k8NByzEV2RfQCSDYhbYBOKwlpGqn7hX9xyHesAs";
    private static final String tfodModelAsset = "UltimateGoal.tflite";
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tfod;

    private MotorMap driveMap;
    private XDrive xDrive;
    private boolean hasMoved[] = {false, false, false, false, false};

    private MotorMap mechMap;
    private ServoMap servoMap;
    private Conveyor conveyor;
    private Shooter shoot;
    private Intake intake;
    private Pushy pushy;


    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        xDrive = new XDrive(driveMap);

        mechMap = new MotorMap(hardwareMap, "conveyor", "intake", "shoot");
        for (DcMotor motor : mechMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        servoMap = new ServoMap(hardwareMap, "pushServo");
        for (Servo servo : servoMap.GetServoMap().values()) {
            servo.setPosition(0);
        }

        conveyor = new Conveyor(mechMap);
        shoot = new Shooter(mechMap);
        intake = new Intake(mechMap);
        pushy = new Pushy(servoMap);
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
            telemetry.addData("hasMoved1", hasMoved[0]);
            hasMoved[0] = xDrive.StrafeByDistance(1000, Math.PI / 2, telemetry);
            telemetry.addData("Move", 1);
        } else if (!hasMoved[1]) {
            telemetry.addData("hasMoved1", hasMoved[1]);
            hasMoved[1] = shoot.Shoot();
            telemetry.addData("Move", 2);
        } else if (!hasMoved[2]) {
            telemetry.addData("hasMoved1", hasMoved[2]);
            pushy.Push();
            hasMoved[2] = true;
            telemetry.addData("Move", 3);
        } else {
            for (DcMotor motor : driveMap.GetMotorMap().values()) {
                motor.setPower(0);
            }
            shoot.SetPower(0);
        }



        /***
        if(!hasMoved[0]) {
            telemetry.addData("hasMoved1", hasMoved[0]);
            hasMoved[0] = xDrive.StrafeByDistance(1000, Math.PI/2, telemetry);
            telemetry.addData("Move", 1);
        } else if (!hasMoved[1]) {
            telemetry.addData("hasMoved1", hasMoved[1]);
            hasMoved[1] = xDrive.StrafeByDistance(1000, Math.PI, telemetry);
            telemetry.addData("Move", 2);
        } else if (!hasMoved[2]) {
            hasMoved[2] = xDrive.StrafeByDistance(1000, Math.PI/3, telemetry);
        } else if (!hasMoved[3]) {
            hasMoved[3] = xDrive.RotateByAngle(Math.PI, true, telemetry);
        }
         **/
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
