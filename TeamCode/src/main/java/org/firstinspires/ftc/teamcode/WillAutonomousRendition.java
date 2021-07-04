package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Arrays;

@Autonomous(name="UltimateGoalWillRendition", group="Autonomous")
public class WillAutonomousRendition extends OpMode {
    private MotorMap driveMap;
    private Robot robot;
    private int ringQuantity = -999;
    private int delta = 0;

    private boolean startMove[] = new boolean[50], hasMove[] = new boolean[50], willMove[] = new boolean[50];
    private boolean isMotorRaise = false;
    @Override
    public void init() {
        driveMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");

        for (DcMotor motor : driveMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        robot = new Robot(hardwareMap, driveMap, "conveyor", "pushy", "intake",
                "shooter", "wobbleLeft", "wobbleRight",
                "clawLeft", "clawRight", "ramp",
                "colorSensorRight1", "colorSensorRight4",
                "colorSensorLeft1", "colorSensorLeft4",
                "wobbleMotor", "wobbleGoalServo");

        Arrays.fill(startMove, false);
        Arrays.fill(hasMove, false);
    }

    @Override
    public void init_loop () {

    }

    @Override
    public void start () {

    }

    @Override
    public void loop() {
        telemetry.addData("ring quantity", ringQuantity);
        telemetry.addData("is the motor rising", isMotorRaise);
        telemetry.addData("target position", robot.getWobbleTargetPosition());
        switch(findFirstInstanceOfFalse(startMove)) {
            case 0:
                startMove[0] = robot.StrafeByDistance(1080, Math.PI/2, telemetry);
                break;
            case 1:
                ringQuantity = robot.GetRedRight();
                startMove[1] = true;
                break;
            case 2:
                startMove[2] = robot.StrafeByDistance(260, Math.PI/2, telemetry);
                break;
            case 3:
                if (ringQuantity == 4) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(1677.98, Math.toRadians(65.3441), telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.RotateByAngleUsingIMU(Math.toRadians(90),true,telemetry);
                            break;
                        default:
                            startMove[3] = true;
                            break;
                    }
                } else if (ringQuantity == 1) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(1378.77, Math.toRadians(71.8279), telemetry);
                            break;
                        default:
                            startMove[3] = true;
                            break;
                    }
                } else {
                    switch(findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(1118.27, Math.toRadians(16.8955), telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.RotateByAngleUsingIMU(Math.toRadians(90),true,telemetry);
                            break;
                        /*case 2:
                            hasMove[2] = robot.StrafeByDistance(1010, Math.PI/2, telemetry);
                            break;*/
                        default:
                            startMove[3] = true;
                            break;
                    }
                }
                break;
            case 4:
                robot.motorLower();
                startMove[4] = true;
                break;
            case 5:
                startMove[5] = robot.Sleep(2000, telemetry);
                break;
            case 6:
                robot.ClawOpen();
                startMove[6] = true;
            case 7:
                startMove[7] = robot.Sleep(1000, telemetry);
                break;
            case 8:// 3 sets of if else statements to reset robot to shooting position
                startMove[8] = robot.StrafeByDistance(1010, 3*Math.PI/2, telemetry);
                break;
                /*robot.ClawClose();
                startMove[7] = true;
                break;
            case 8:
                robot.motorRaise();
                isMotorRaise = true;
                startMove[8] = true;
                break;*/
            case 9:
                startMove[9] = robot.RotateByAngleUsingIMU(Math.toRadians(90),false,telemetry);
                break;
            /*case 9:
                if (ringQuantity == 4) {
                    switch (findFirstInstanceOfFalse(willMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(2070, Math.PI/2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(900, Math.PI * 2, telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                } else if (ringQuantity == 1) {
                    switch (findFirstInstanceOfFalse(willMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(1500, Math.PI / 2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(600, Math.PI * 2, telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                } else {
                    switch(findFirstInstanceOfFalse(willMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(235, Math.PI/2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.RotateByAngleUsingIMU(Math.toRadians(90),true,telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                }
                break;*/

            case 10:
                robot.Shoot(10);
                startMove[10] = true;
                break;
            case 11:
                robot.PushThenRetract(telemetry);
                startMove[11] = robot.Sleep(1000, telemetry);
                break;
            case 12:
                robot.PushThenRetract(telemetry);
                startMove[12] = robot.Sleep(1000, telemetry);
                break;
            case 13:
                robot.PushThenRetract(telemetry);
                startMove[13] = true;
                break;
            default:
                robot.SetStrafe(0,0);
                robot.SetRotation(0);
                break;
        }
    }

    @Override
    public void stop () {
        robot.setWobbleGoalToZero();
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
