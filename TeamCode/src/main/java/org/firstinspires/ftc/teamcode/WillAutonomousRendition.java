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
    private int ringQuantity;
    private int delta = 0;

    private boolean startMove[] = new boolean[50], hasMove[] = new boolean[50];

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
        switch(findFirstInstanceOfFalse(startMove)) {
            case 0:
                startMove[0] = robot.StrafeByDistance(1371, Math.PI/2, telemetry);
                break;
            case 1:
                ringQuantity = robot.GetRedLeft();
                startMove[1] = true;
                break;
            case 2:
                if (ringQuantity == 4) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(3*22.5*25.4, Math.PI/2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(22.5*25.4, Math.PI, telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                } else if (ringQuantity == 1) {
                    switch (findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(2 * 22.5*25.4, Math.PI / 2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(0.5*22.5*25.4, Math.PI, telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                } else {
                    switch(findFirstInstanceOfFalse(hasMove)) {
                        case 0:
                            hasMove[0] = robot.StrafeByDistance(22.5*25.4, Math.PI/2, telemetry);
                            break;
                        case 1:
                            hasMove[1] = robot.StrafeByDistance(22.5*25.4, Math.PI, telemetry);
                            break;
                        default:
                            startMove[2] = true;
                            break;
                    }
                }
                break;
            case 3:
                robot.motorLower();
                startMove[3] = true;
                break;
            case 4:
                startMove[4] = robot.Sleep(100, telemetry);
                break;
            case 5:
                robot.ClawOpen();
                startMove[5] = true;
            case 6:
                startMove[6] = robot.Sleep(100, telemetry);
                break;
            case 7:
                robot.ClawClose();
                startMove[7] = true;
                break;
            case 8:
                robot.motorRaise();
                startMove[8] = true;
                break;
            case 9:
                robot.StrafeByDistance(1000, 2*Math.PI - 7/36 * Math.PI, telemetry);
            default:
                robot.SetStrafe(0,0);
                robot.SetRotation(0);
                break;
        }
    }

    @Override
    public void stop () {

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
