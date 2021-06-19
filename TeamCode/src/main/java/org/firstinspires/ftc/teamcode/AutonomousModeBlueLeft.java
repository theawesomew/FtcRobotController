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

@Autonomous(name="UltimateGoalAutonomousModeBlueLeft", group="Autonomous")
public class AutonomousModeBlueLeft extends OpMode {
    private static final String vuforiaKey = "AV98RL7/////AAABmYXDdoEmIUn1v6Ppa4Z/oN0tDQt5nJk+KD9Gy+XiJe1DpevozXumH++UVyRGG8Al6PX2as4EddLKpGncqMsiDQeugSuOXBAKBVnpGda6+GX6veXRgYkOEwq4HDxSPi3Nfqoe8/6GVo0TH5sqyOfCgZLIk2o2rzjmrrCbcou31JRGpg25elDXgbtXQcD+qPq748IrnJLh7/vbsk9tBANafFczL8l2mesx8Rj8i00T3x9JIHqPku9j3cUReAzTxa6X7vq/5IC2AtS05lFjmjlNkJRgnxVAwBjAgFtYBH2O8eXGUtY147+ABdxJLpmIbeOZDvZ38k8NByzEV2RfQCSDYhbYBOKwlpGqn7hX9xyHesAs";
    private static final String tfodModelAsset = "UltimateGoal.tflite";
    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector tfod;

    private VoltageSensor voltageSensor;

    private MotorMap driveMap;
    private Robot robot;
    private boolean hasMoved[] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    private boolean startMove[] = {false, false, false, false, false, false, false};
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
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobble", "clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4");

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

        telemetry.addData("Red Value1: ", red1);
        telemetry.addData("Red Value2: ", red2);
        telemetry.addData("Rings Detected", ringsDetected);
        telemetry.addData("Shooter Power", robot.GetShooterPower());



        switch (findFirstInstanceOfFalse(startMove)) {
            case 0:
                startMove[0] = robot.StrafeByDistance(1200, Math.PI/2, telemetry);
                break;
            case 1:
                startMove[1] = robot.Sleep(1000, telemetry);
                break;
            case 2:
                ringsDetected = robot.GetRed();
                startMove[2] = true;
                break;
            case 3:
                startMove[3] = robot.Sleep(1000, telemetry);
                telemetry.addData("Currently", "Stopped");
                break;
            case 4:
                startMove[4] = robot.RotateByAngleUsingIMU(Math.toRadians(2), false, telemetry);
            case 5:
                if (ringsDetected == 1) {
                    telemetry.addData("working", ":)");
                    switch (findFirstInstanceOfFalse(hasMoved)) {
                        case 0:
                            hasMoved[0] = robot.Up();
                            break;
                        case 1:
                            telemetry.addData("Moving now for has moved 1", "pog");
                            hasMoved[1] = robot.StrafeByDistance(1200, Math.PI/2, telemetry);
                            break;
                        case 2:
                            hasMoved[2] = robot.RotateByAngleUsingIMU(Math.toRadians(70), true, telemetry);
                            break;
                        case 3:
                            hasMoved[3] = robot.StrafeByDistance(5, Math.PI/2, telemetry);
                            break;
                        case 4:
                            robot.Lower();
                            hasMoved[4] = robot.Sleep(2000, telemetry);
                            break;
                        case 5:
                            robot.ClawOpen();
                            hasMoved[5] = true;
                            break;
                        case 6:
                            robot.Raise();
                            hasMoved[6] = robot.Sleep(2000, telemetry);
                            break;
                        case 7:
                            hasMoved[7] = robot.StrafeByDistance(350, 0, telemetry);
                            break;
                        case 8:
                            hasMoved[8] = robot.Shoot(10);
                            break;
                        case 9:
                            hasMoved[9] = robot.RotateByAngleUsingIMU(Math.toRadians(65), false, telemetry);
                            break;
                        case 10:
                            try {
                                hasMoved[10] = robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 11:
                            hasMoved[11] = robot.StrafeByDistance(100, 0 - Math.toRadians(5), telemetry);
                            break;
                        case 12:
                            try {
                                hasMoved[12] = robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 13:
                            hasMoved[13] = robot.StrafeByDistance(5, 0 - robot.GetYaw(), telemetry);
                            break;
                        case 14:
                            try {
                                hasMoved[14] = robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            startMove[5] = true;
                            break;
                    }

                } else {
                    startMove[5] = true;
                    break;
                }
                break;
            case 6:
                if (ringsDetected == 0) {
                    switch (findFirstInstanceOfFalse(hasMoved)) {
                        case 0:
                            telemetry.addData("zero Rings found", "No code for here atm");
                            break;
                    }
                } else {
                    startMove[6] = true;
                    break;
                }
            case 7:
                if (ringsDetected == 4) {
                    switch (findFirstInstanceOfFalse(hasMoved)) {
                        case 0:
                            telemetry.addData("Four rings found", ":)");
                            break;
                    }
                } else {
                    startMove[7] = true;
                    break;
                }

        }


        // no rings, square a
            /**

        switch (findFirstInstanceOfFalse(hasMoved)) {
            case 0:
                hasMoved[0] = robot.Up();
                break;

            case 1:
                hasMoved[1] = robot.StrafeByDistance(700, Math.PI/2, telemetry);
                break;
            case 2:
                hasMoved[2] = robot.LoweredPosition();
                break;
            case 3:
                robot.ClawOpen();
                hasMoved[3] = true;
                break;
            case 4:
                hasMoved[4] = robot.RaisedPosition();
                break;
            case 5:
                hasMoved[5] = robot.StrafeByDistance(350, 3*Math.PI/2, telemetry);
                break;
            case 6:
                hasMoved[6] = robot.Shoot(10000);
                break;
            case 7:
                hasMoved[7] = robot.StrafeByDistance(350, 0, telemetry);
                break;
            case 8:
                hasMoved[8] = robot.RotateByAngleUsingIMU(Math.toRadians(15), true, telemetry);
                break;
            case 9:
                try {
                    hasMoved[9] = robot.PushThenRetract(telemetry);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                break;
            case 10:
                hasMoved[10] = robot.StrafeByDistance(5, 0 - robot.GetYaw(), telemetry);
                break;
            case 11:
                try {
                    hasMoved[11] = robot.PushThenRetract(telemetry);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                break;
            case 12:
                hasMoved[12] = robot.StrafeByDistance(5, 0 - robot.GetYaw(), telemetry);
                break;
            case 13:
                try {
                    hasMoved[13] = robot.PushThenRetract(telemetry);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                break; */
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
