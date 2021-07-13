package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.MotorMapEx;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Arrays;

@Autonomous(name="BACKUPPLAN", group="Autonomous")
public class BACKUPPLAN extends OpMode {
    private MotorMap driveMap;
    private MotorMapEx motorMapEx;
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

        motorMapEx = new MotorMapEx(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");
        for (DcMotorEx motor : motorMapEx.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot = new Robot(hardwareMap, driveMap, motorMapEx, "conveyor", "pushy", "intake",
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
                startMove[0] = robot.StrafeByDistance(3800,Math.toRadians(100), telemetry);
                telemetry.addData("Step", "one");
                break;
            case 1:
                robot.Shoot(1000);
                startMove[2] = true;
                break;
            case 2:
                startMove[1] = robot.StrafeByDistance(1000, Math.PI, telemetry);
                telemetry.addData("Step", "two");
                break;
            case 3:
                startMove[3] = true;//robot.RotateByAngleUsingIMU(Math.toRadians(1), false, telemetry );
                telemetry.addData("Step", "three");
                break;
            case 4:
                robot.PushThenRetract(telemetry);
                startMove[4] = robot.Sleep(1000, telemetry);
                break;
            case 5:
                robot.PushThenRetract(telemetry);
                startMove[5] = robot.Sleep(1000, telemetry);
                break;
            case 6:
                robot.PushThenRetract(telemetry);
                startMove[6] = robot.Sleep(1000, telemetry);
                break;
            case 7:
                startMove[7] = robot.StrafeByDistance(500, Math.PI/2, telemetry);
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
