package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double distance;
    private double minDistance = 70;
    private boolean ringBool = true;
    private boolean ringTimer = false;
    private TextToSpeech tts;



    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");  //front left = white tape, back left = red tape, front right = blue, back right = black
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        ringSensor = hardwareMap.get(DistanceSensor.class, "ringSensor");

        robot = new Robot(hardwareMap, motorMap, "conveyor", "pushy", "intake", "shooter", "wobbleLeft", "wobbleRight","clawLeft", "clawRight", "ramp", "colorSensorRight1", "colorSensorRight4", "wobbleMotor");
        gamepadWrapper = new GamepadWrapper();

        tts = new TextToSpeech(hardwareMap.appContext, null);
        tts.setLanguage(Locale.US);


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


        if (gamepadWrapper.isDown("g1_right_trigger")) {
            robot.AdjustedShootPower();
            telemetry.addData("Shooting Power:", robot.GetShooterPower());
        } else {
            robot.SetShooterPower(0);
        }

        if (ringBool == true) {
            if (gamepadWrapper.isPressed("g2_dpad_down")) {
                robot.SetIntakePower(++IntakeConveyorCounter % 2);
                robot.SetConveyorPower(IntakeConveyorCounter % 2);
            }
        }

        if (distance < minDistance) {
            ringTimer = robot.IntakeSleep(2000, telemetry);
            if (ringTimer == true ) {
                tts.speak("Three rings detected", TextToSpeech.QUEUE_FLUSH, null);
                ringBool = false;
            } else {
                ringBool = true;
            }


        }



        if (gamepadWrapper.isDown("g1_a")) {
            robot.Push();
        } else {
            robot.Retract();
        }

        if (gamepadWrapper.isPressed("g2_a")) {
            ++WobbleArmCounter;
            if (WobbleArmCounter % 2 == 0) {
                robot.motorRaise();
            } else {
                robot.motorLower();
            }
        }

        if (gamepadWrapper.isDown("g1_y")) {
            try {
                robot.PushThenRetract(telemetry);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
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







        telemetry.addData("Yaw", robot.GetYaw());
        telemetry.addData("Roll", robot.GetRoll());
        telemetry.addData("Pitch", robot.GetPitch());

        telemetry.addData("range", String.format("%.01f mm", ringSensor.getDistance(DistanceUnit.MM)));

    }

    @Override
    public void stop() {

    }
}
