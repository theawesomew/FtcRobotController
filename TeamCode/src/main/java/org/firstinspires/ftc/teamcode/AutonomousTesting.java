package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="AutonomousTesting", group="Autonomous")
public class AutonomousTesting extends OpMode {
    private static final String vuforiaKey = "AV98RL7/////AAABmYXDdoEmIUn1v6Ppa4Z/oN0tDQt5nJk+KD9Gy+XiJe1DpevozXumH++UVyRGG8Al6PX2as4EddLKpGncqMsiDQeugSuOXBAKBVnpGda6+GX6veXRgYkOEwq4HDxSPi3Nfqoe8/6GVo0TH5sqyOfCgZLIk2o2rzjmrrCbcou31JRGpg25elDXgbtXQcD+qPq748IrnJLh7/vbsk9tBANafFczL8l2mesx8Rj8i00T3x9JIHqPku9j3cUReAzTxa6X7vq/5IC2AtS05lFjmjlNkJRgnxVAwBjAgFtYBH2O8eXGUtY147+ABdxJLpmIbeOZDvZ38k8NByzEV2RfQCSDYhbYBOKwlpGqn7hX9xyHesAs";
    private static final String tfodModelAsset = "UltimateGoal.tflite";
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tfod;

    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}; //27 falses
    private boolean startMove[] = {false, false, false, false, false, false, false, false, false, false}; //10 falses
    private double currentYaw;

    private int ringsDetected = 0;

    private ColorSensor right1;
    private ColorSensor right2;

    private int red1;
    private int red2;





    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight","clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4","colorSensorLeft1", "colorSensorLeft4",  "wobbleMotor", "wobbleGoalServo");

        robot.ClawClose();
        right1 = hardwareMap.get(ColorSensor.class, "colorSensorRight1");
        right2 = hardwareMap.get(ColorSensor.class, "colorSensorRight4");

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


        // autonomous blue left
        switch (findFirstInstanceOfFalse(startMove)) {
            case 0:
                startMove[0] = robot.StrafeByDistance(1140, Math.PI/2, telemetry);
                break;
            case 1:
                startMove[1] = robot.Sleep(500, telemetry);
                break;
            case 2:
                ringsDetected = robot.GetRedRight();
                startMove[2] = true;
                break;
            case 3:
                startMove[3] = robot.Sleep(250, telemetry);
                telemetry.addData("Currently", "Stopped");
                break;
            case 4:
                startMove[4] = true; // robot.RotateByAngleUsingIMU(Math.toRadians(2), false, telemetry);
                break;
            case 5:
                if (ringsDetected == 0) {
                    switch (findFirstInstanceOfFalse(hasMoved)) {
                        case 0:
                            hasMoved[0] = robot.StrafeByDistance(900, Math.PI/2, telemetry);
                            break;
                        case 1:
                            robot.motorLower();
                            hasMoved[1] = robot.Sleep(1500, telemetry);
                            break;
                        case 2:
                            robot.ClawOpen();
                            hasMoved[2] = true;
                            break;
                        case 3:
                            robot.motorRaise();
                            hasMoved[3] = robot.Sleep(1500, telemetry);
                            break;
                        case 4:
                            hasMoved[4] = robot.StrafeByDistance(780, -Math.toRadians(35.06), telemetry);
                            robot.Shoot(11);
                            break;
                        case 5:
                            hasMoved[5] = robot.PushThenRetract(telemetry);
                            break;
                        case 6:
                            robot.PushThenRetract(telemetry);
                            hasMoved[6] = robot.Sleep(500, telemetry);

                            break;
                        case 7:
                            hasMoved[7] = robot.PushThenRetract(telemetry);
                            break;

                    }

                }
                break;




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
}

