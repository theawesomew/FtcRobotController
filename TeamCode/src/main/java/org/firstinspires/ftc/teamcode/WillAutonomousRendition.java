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
        telemetry.addData("sensor output :  ambient", ringQuantity);
        telemetry.addData("is the motor rising", isMotorRaise);
        telemetry.addData("target position", robot.getWobbleTargetPosition());
        switch(findFirstInstanceOfFalse(startMove)) {
            case 0:
                //for leter
                // startMove[0] = robot.StrafeByDistance(1084.62 , Math.toRadians(84.7099), telemetry);
                //startMove[0] = robot.StrafeByDistance(1080 , Math.toRadians(90), telemetry);
                startMove[0] = robot.StrafeByDistance(1480 , Math.toRadians(90), telemetry);
                break;
            case 1:
                startMove[1] = robot.StrafeByDistance(545, Math.toRadians(200), telemetry);
                break;
            case 2:
                startMove[2] = robot.RotateByAngleUsingIMU(Math.toRadians(5),false,telemetry);
                break;
            case 3:
                ringQuantity = robot.GetRedLeft();
                //ringQuantity = 4;
                startMove[3] = true;
                break;
            case 4:
                startMove[4] = robot.Shoot(1000);
                break;
            case 5:
                startMove[5] = robot.StrafeByDistance(800, Math.toRadians(100), telemetry);
                break;
            case 6:
                startMove[6] = robot.StrafeByDistance(200,Math.PI*2,telemetry);
                break;
            case 7:
                startMove[7] = robot.RotateByAngleUsingIMU(Math.toRadians(15),true,telemetry);
                break;
            case 8:
                robot.PushThenRetract(telemetry);
                startMove[8] = robot.Sleep(1000, telemetry);
                break;
            case 9:
                robot.PushThenRetract(telemetry);
                startMove[9] = robot.Sleep(1000, telemetry);
                break;
            case 10:
                robot.PushThenRetract(telemetry);
                startMove[10] = robot.Sleep(1000, telemetry);
                break;
            case 11:
                if (ringQuantity == 4) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.RotateByAngleUsingIMU(Math.toRadians(2),false,telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(1800,Math.PI/2, telemetry);
                            break;
                        case 2:
                            hasMove[2] = robot.RotateByAngleUsingIMU(Math.toRadians(180),false,telemetry);
                            break;
                        default:
                            startMove[11] = true;
                            break;
                    }
                } else if (ringQuantity == 1) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(1200,Math.PI/2, telemetry);
                            break;
                        default:
                            startMove[11] = true;
                            break;
                    }
                } else {
                    switch(findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(400, Math.PI/2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.RotateByAngleUsingIMU(Math.toRadians(180),false,telemetry);
                            break;
                        case 2:
                            hasMove[2]= robot.Sleep(500, telemetry);
                            break;
                        default:
                            startMove[11] = true;
                            break;
                    }
                }
                break;
            case 12:
                robot.motorLower();
                startMove[12] = true;
                break;
            case 13:
                startMove[13] = robot.Sleep(1000, telemetry);
                break;
            case 14:
                robot.ClawOpen();
                startMove[14] = true;
            case 15:
                startMove[15] = robot.Sleep(1000, telemetry);//robot.Shoot(1000);
                break;
            case 16:// 3 sets of if else statements to reset robot to shooting position
                if (ringQuantity == 4) {
                    switch (findFirstInstanceOfFalse(willMove)) {
                        case 0:
                            willMove[0] = robot.StrafeByDistance(1400, Math.PI/2, telemetry);
                            break;
                        default:
                            startMove[16] = true;
                            break;
                    }
                } else if (ringQuantity == 1) {
                    switch (findFirstInstanceOfFalse(willMove)) {
                        case 0://change distance here
                            willMove[0] = robot.StrafeByDistance(900, 3 * Math.PI / 2, telemetry);
                            break;
                        default:
                            startMove[16] = true;
                            break;
                        }
                    }else {
                        switch(findFirstInstanceOfFalse(hasMove)) {
                            case 0:
                                hasMove[0] = true;
                                break;
                            default:
                                startMove[11] = true;
                                break;
                    }
                }
                break;
            case 17://remove later if needed
                robot.motorRaise();
                startMove[17] = true;
                break;
            case 18:
                startMove[18] = robot.RotateByAngleUsingIMU(Math.toRadians(180),true,telemetry);
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
