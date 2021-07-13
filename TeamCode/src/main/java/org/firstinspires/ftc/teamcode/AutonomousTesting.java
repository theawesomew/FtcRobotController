package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.MotorMapEx;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="AutonomousTesting", group="Autonomous")
public class AutonomousTesting extends OpMode {
    private static final String vuforiaKey = "AV98RL7/////AAABmYXDdoEmIUn1v6Ppa4Z/oN0tDQt5nJk+KD9Gy+XiJe1DpevozXumH++UVyRGG8Al6PX2as4EddLKpGncqMsiDQeugSuOXBAKBVnpGda6+GX6veXRgYkOEwq4HDxSPi3Nfqoe8/6GVo0TH5sqyOfCgZLIk2o2rzjmrrCbcou31JRGpg25elDXgbtXQcD+qPq748IrnJLh7/vbsk9tBANafFczL8l2mesx8Rj8i00T3x9JIHqPku9j3cUReAzTxa6X7vq/5IC2AtS05lFjmjlNkJRgnxVAwBjAgFtYBH2O8eXGUtY147+ABdxJLpmIbeOZDvZ38k8NByzEV2RfQCSDYhbYBOKwlpGqn7hX9xyHesAs";
    private static final String tfodModelAsset = "UltimateGoal.tflite";
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tfod;

    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private MotorMapEx motorMapEx;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; //27 falses
    private boolean startMove[] = {false, false, false, false, false, false, false, false, false, false}; //10 falses
    private double currentYaw;

    private int ringsDetected = 0;

    private ColorSensor right1;
    private ColorSensor right2;

    private int red1;
    private int red2;

    public DcMotor forwardRight = hardwareMap.dcMotor.get("forwardRight");
    public DcMotor forwardLeft = hardwareMap.dcMotor.get("forwardLeft");
    public DcMotor backRight = hardwareMap.dcMotor.get("backRight");
    public DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");




    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        motorMapEx = new MotorMapEx(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");
        for (DcMotorEx motor : motorMapEx.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, motorMapEx, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight","clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4","colorSensorLeft1", "colorSensorLeft4",  "wobbleMotor", "wobbleGoalServo");

        robot.ClawClose();
        right1 = hardwareMap.get(ColorSensor.class, "colorSensorRight1");
        right2 = hardwareMap.get(ColorSensor.class, "colorSensorRight4");

        forwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        forwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
         * To write code in this instance, simply write
         * case n:
         *   hasMoved[n] = *some instruction*
         *   break;
         *  */

        red1 = right1.red();
        red2 = right2.red();

        telemetry.addData("Yaw", robot.GetYaw());

        telemetry.addData("Red Value1: ", red1);
        telemetry.addData("Red Value2: ", red2);
        telemetry.addData("Rings Detected", ringsDetected);
        telemetry.addData("Shooter Power", robot.GetShooterPower());


        forwardDrive(1000);







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

    private int findFirstInstanceOfFalse (boolean[] array) {
        if (!array[0]) return 0;

        int lb = 0, rb = array.length - 1;
        while (lb < rb) {
            int mid = (lb+rb)/2;
            if (mid+1 >= array.length) break;
            if (array[mid] == array[mid+1]) {
                if (array[mid]) {
                    lb = mid+1;
                } else {
                    rb = mid;
                }
            } else {
                return mid+1;
            }
        }
        return -1;
    }

    private final void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void forwardDrive(int bedtime) {
        backRight.setPower(-0.5);
        forwardRight.setPower(0.5);
        forwardLeft.setPower(0.5);
        backLeft.setPower(-0.5);
        sleep(bedtime);
    }
}

