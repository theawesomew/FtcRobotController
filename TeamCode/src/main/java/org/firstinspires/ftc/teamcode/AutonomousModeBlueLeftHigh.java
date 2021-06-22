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

@Autonomous(name="UltimateGoalAutonomousModeBlueLeftHigh", group="Autonomous")
public class AutonomousModeBlueLeftHigh extends OpMode {
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
        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight","clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4");

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
                startMove[1] = robot.Sleep(500, telemetry);
                break;
            case 2:
                ringsDetected = robot.GetRed();
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
                            hasMoved[0] = robot.StrafeByDistance(1100, Math.PI/2, telemetry);
                            break;
                        case 1:
                            try {
                                robot.Lower();
                                hasMoved[1] = robot.Sleep(10, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            break;
                        case 2:
                            robot.ClawOpen();
                            hasMoved[2] = robot.Sleep(10, telemetry);
                            break;
                        case 3:
                            try {
                                robot.Raise();
                                hasMoved[3] = robot.Sleep(10, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            break;
                        case 4:
                            hasMoved[4] = robot.StrafeByDistance(1200, 0, telemetry);
                            break;
                        case 5:
                            robot.Shoot(10);
                            hasMoved[5] = robot.StrafeByDistance(500, -Math.PI/2, telemetry);
                            break;
                        case 6:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[6] = robot.Sleep(10, telemetry);
                            break;
                        case 7:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[7] = robot.Sleep(10, telemetry);
                            break;
                        case 8:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[8] = true;
                            break;
                        case 9:
                            hasMoved[9] = robot.RotateByAngleUsingIMU(Math.toRadians(90), false, telemetry);
                            break;
                        case 10:
                            hasMoved[10] = robot.StrafeByDistance(1500, Math.PI, telemetry);
                            break;
                        case 11:
                            robot.ClawClose();
                            hasMoved[11] = robot.Sleep(10, telemetry);
                            break;
                        case 12:
                            try {
                                robot.Raise();
                                hasMoved[12] = robot.Sleep(10, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 13:
                            hasMoved[13] = robot.RotateByAngleUsingIMU(Math.toRadians(90), true, telemetry);
                            break;
                        case 14:
                            hasMoved[14] = robot.StrafeByDistance(2200, Math.PI/2, telemetry);
                            break;
                        case 15:
                            try {
                                robot.Lower();
                                hasMoved[15] = robot.Sleep(10, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 16:
                            robot.ClawOpen();
                            hasMoved[16] = robot.Sleep(10, telemetry);
                            break;
                        case 17:
                            hasMoved[17] = true;
                            startMove[5] = true;
                            break;
                    }

                } else if (ringsDetected == 1) {
                   switch (findFirstInstanceOfFalse(hasMoved)) {
                       case 0:
                           hasMoved[0] = robot.StrafeByDistance(1600, Math.PI/2, telemetry);
                           break;
                       case 1:
                           hasMoved[1] = robot.StrafeByDistance(500, 0, telemetry);
                           break;
                       case 2:
                           try {
                               robot.Lower();
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[2] = robot.Sleep(10, telemetry);
                           break;
                       case 3:
                           robot.ClawOpen();
                           hasMoved[3] = robot.Sleep(10, telemetry);
                           break;
                       case 4:
                           try {
                               robot.Raise();
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[4] = robot.Sleep(10, telemetry);
                           break;
                       case 5:
                           robot.Shoot(10);
                           hasMoved[5] = robot.StrafeByDistance(700, -Math.PI/2, telemetry);
                           break;
                       case 6:
                           try {
                               robot.PushThenRetract(telemetry);
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[6] = robot.Sleep(10, telemetry);
                           break;
                       case 7:
                           try {
                               robot.PushThenRetract(telemetry);
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[7] = robot.Sleep(10, telemetry);
                           break;
                       case 8:
                           try {
                               robot.PushThenRetract(telemetry);
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[8] = robot.Sleep(10, telemetry);
                           break;
                       case 9:
                           hasMoved[9] = robot.RotateByAngleUsingIMU(Math.toRadians(90), false, telemetry);
                           break;
                       case 10:
                           try {
                               robot.Lower();
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[10] = robot.Sleep(10, telemetry);
                           break;
                       case 11:
                           hasMoved[11] = robot.StrafeByDistance(1500, Math.PI, telemetry);
                           break;
                       case 12:
                           robot.ClawClose();
                           hasMoved[12] = robot.Sleep(10, telemetry);
                           break;
                       case 13:
                           try {
                               robot.Raise();
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[13] = robot.Sleep(10, telemetry);
                           break;
                       case 14:
                           hasMoved[14] = robot.RotateByAngleUsingIMU(Math.toRadians(90), true, telemetry);
                           break;
                       case 15:
                           hasMoved[15] = robot.StrafeByDistance(2800, Math.PI/2, telemetry);
                           break;
                       case 16:
                           try {
                               robot.Lower();
                           } catch (InterruptedException e) {
                               e.printStackTrace();
                           }
                           hasMoved[16] = robot.Sleep(10, telemetry);
                           break;
                       case 17:
                           robot.ClawOpen();
                           hasMoved[17] = robot.Sleep(10, telemetry);
                           break;
                       case 18:
                           hasMoved[18] = robot.StrafeByDistance(500, -Math.PI/2, telemetry);
                           break;
                       case 19:
                           hasMoved[19] = true;
                           startMove[5] = true;
                           break;


                   }

                } else if (ringsDetected == 4) {
                    switch (findFirstInstanceOfFalse(hasMoved)) {
                        case 0:
                            hasMoved[0] = robot.StrafeByDistance(3500, Math.PI/2, telemetry);
                            break;
                        case 1:
                            try {
                                robot.Lower();
                                hasMoved[1] = robot.Sleep(100, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            break;
                        case 2:
                            robot.ClawOpen();
                            hasMoved[2] = robot.Sleep(100, telemetry);
                            break;
                        case 3:
                            try {
                                robot.Raise();
                                hasMoved[3] = robot.Sleep(100, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            break;
                        case 4:
                            hasMoved[4] = robot.StrafeByDistance(1200, 0, telemetry);
                            break;
                        case 5:
                            robot.Shoot(10);
                            hasMoved[5] = robot.StrafeByDistance(2900, -Math.PI/2, telemetry);
                            break;
                        case 6:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[6] = robot.Sleep(10, telemetry);
                            break;
                        case 7:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[7] = robot.Sleep(10, telemetry);
                            break;
                        case 8:
                            try {
                                robot.PushThenRetract(telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[8] = true;
                            break;
                        case 9:
                            hasMoved[9] = robot.RotateByAngleUsingIMU(Math.toRadians(90), false, telemetry);
                            break;
                        case 10:
                            hasMoved[10] = robot.StrafeByDistance(1500, Math.PI, telemetry);
                            break;
                        case 11:
                            robot.ClawClose();
                            hasMoved[11] = robot.Sleep(10, telemetry);
                            break;
                        case 12:
                            try {
                                robot.Raise();
                                hasMoved[12] = robot.Sleep(10, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 13:
                            hasMoved[13] = robot.RotateByAngleUsingIMU(Math.toRadians(90), true, telemetry);
                            break;
                        case 14:
                            hasMoved[14] = robot.StrafeByDistance(5000, Math.PI/2, telemetry);
                            break;
                        case 15:
                            try {
                                robot.Lower();
                                hasMoved[15] = robot.Sleep(100, telemetry);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            break;
                        case 16:
                            robot.ClawOpen();
                            hasMoved[16] = robot.Sleep(100, telemetry);
                            break;
                        case 17:
                            try {
                                robot.Raise();
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            hasMoved[17] = robot.Sleep(100, telemetry);
                            break;
                        case 18:
                            robot.StrafeByDistance(2900, -Math.PI/2, telemetry);
                            break;
                        case 19:
                            hasMoved[17] = true;
                            startMove[5] = true;
                            break;

                    }
                }
            break;




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
