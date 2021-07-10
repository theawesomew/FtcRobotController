package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.GamepadWrapper;
import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Pushy;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoMap;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.XDrive;

import java.util.Locale;


@TeleOp(name="UltimateGoalTeleOpMode", group="LinearOpMode")
public class TeleOpMode extends OpMode {
    private MotorMap motorMap;
    private GamepadWrapper gamepadWrapper;
    private Robot robot;
    private int IntakeConveyorCounter = 0;
    private int WobbleArmCounter = 0;
    private int ClawCounter = 0;
    private int RampCounter = 0;
    private DistanceSensor ringSensor;
    private DistanceSensor wobbleGoalSensor;
    private double distance;
    private double minDistance = 50;
    private double maxDistance = 70;
    private int rings = 0;
    private boolean ringCounted = false;
    private boolean wobbleGoalCurrentlyActive = false;
    private boolean  shooting = false;
    private boolean hasShoot[] = {false, false, false, false};

    private ElapsedTime robotTime;
    private double prevTime = -1.0;


    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");  //front left = white tape, back left = red tape, front right = blue, back right = black
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        ringSensor = hardwareMap.get(DistanceSensor.class, "ringSensor");
        wobbleGoalSensor = hardwareMap.get(DistanceSensor.class, "wobbleGoalSensor");

        robot = new Robot(hardwareMap, motorMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight","clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4","colorSensorLeft1", "colorSensorLeft4", "wobbleMotor", "wobbleGoalServo");
        gamepadWrapper = new GamepadWrapper();

        robotTime = new ElapsedTime();
        robotTime.reset();


    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start () {

    }

    @Override
    public void loop () {
        gamepadWrapper.updateGamepadInputs(gamepad1, gamepad2);
        double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double power = Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y+gamepad1.left_stick_x*gamepad1.left_stick_x);
        robot.SetStrafe(power, angle);
        robot.SetRotation(gamepad1.right_stick_x);
        robot.Drive(telemetry);

        distance = ringSensor.getDistance(DistanceUnit.MM);
        telemetry.addData("arm position", robot.getWobbleArmPosition());

        if (robot.getWobbleArmPosition() >= 3800 && gamepad2.left_stick_y < 0) {
            robot.setWobbleGoalPower(0);
        } else if (robot.getWobbleArmPosition() <= -3800 && gamepad2.left_stick_y > 0) {
            robot.setWobbleGoalPower(0);
        } else {
            robot.setWobbleGoalPower(-gamepad2.left_stick_y);
        }

        if (gamepadWrapper.isDown("g1_right_trigger")) {
            robot.AdjustedShootPower();
            telemetry.addData("Shooting Power:", robot.GetShooterPower());
        } else {
            robot.SetShooterPower(0);
        }


        if (gamepadWrapper.isPressed("g2_dpad_down") && rings < 3) {
            robot.SetIntakePower(++IntakeConveyorCounter % 2);
            robot.SetConveyorPower(IntakeConveyorCounter % 2);
        }


        if (distance > minDistance && distance < maxDistance) {
            if (ringCounted == false) {
                rings += 1;
                ringCounted = true;
            }
        } else {
            ringCounted = false;
        }

        if (rings >= 3) {
            robot.SetIntakePower(0);
            robot.SetConveyorPower(0);
        }


        if (gamepadWrapper.isDown("g1_a")) {
            robot.Push();
            //rings = rings - 1 >= 0 ? rings - 1 : 0;
        } else {
            robot.Retract();
        }

        /*if (gamepadWrapper.isPressed("g2_a")) {
            ++WobbleArmCounter;
            if (WobbleArmCounter % 2 == 0) {
                robot.motorRaise();
                wobbleGoalCurrentlyActive = true;
            } else {
                robot.motorLower();
                wobbleGoalCurrentlyActive = true;
            }
        }*/

        /*if (wobbleGoalCurrentlyActive) {
            if (wobbleGoalSensor.getDistance(DistanceUnit.MM) <= 3) {
                robot.WobbleGoalServoActivate();
            }
        } else {
            robot.WobbleGoalServoDeactive();
        }*/

        if (gamepadWrapper.isDown("g1_y")) {
            robot.PushThenRetract(telemetry);
        }

        if (gamepadWrapper.isPressed("g2_b")) {
            ++ClawCounter;
            if (ClawCounter % 2 == 0) {
                robot.ClawOpen();
            } else {
                robot.ClawClose();
            }

        }

        if (gamepadWrapper.isPressed("g1_b")) {
            ++RampCounter;
            if (RampCounter % 2 == 0) {
                robot.Return();
            } else {
                robot.Extend();
            }
        }
/*
        if (gamepadWrapper.isPressed("g1_x")) {
            shooting = true;
        }

        if (shooting = true) {
            if (!hasShoot[0]) {
                robot.Shoot(5);
                hasShoot[0] = true;
            } else if (!hasShoot[1]) {
                robot.PushThenRetract(telemetry);
                hasShoot[1] = ShootSleep(1000, telemetry);
            } else if (!hasShoot[2]) {
                robot.PushThenRetract(telemetry);
                hasShoot[2] = ShootSleep(1000, telemetry);
            } else if (!hasShoot[3]) {
                robot.PushThenRetract(telemetry);
                hasShoot[3] = ShootSleep(1000, telemetry);
                shooting = false;
            }
        }
*/


        telemetry.addData("Yaw", robot.GetYaw());
        telemetry.addData("Roll", robot.GetRoll());
        telemetry.addData("Pitch", robot.GetPitch());
        telemetry.addData("range", String.format("%.01f mm", ringSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("Rings" , rings);
    }

    @Override
    public void stop() {
        telemetry.addData("bleh", "asdkasjdklasj");
        robot.setWobbleGoalToZero();
    }



    public boolean ShootSleep (double milliseconds, Telemetry telemetry) {
        if (prevTime == -1.0) {
            prevTime = robotTime.milliseconds();
            return false;
        } else if (robotTime.milliseconds() - prevTime < milliseconds) {
            telemetry.addData("Timer", robotTime.milliseconds());
            telemetry.addData("Previous Time", prevTime);
            return false;
        } else {
            prevTime = -1.0;
            return true;
        }
    }
}
