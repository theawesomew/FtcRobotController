package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.GamepadWrapper;
import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Pushy;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.ServoMap;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.XDrive;


@TeleOp(name="UltimateGoalTeleOpMode", group="LinearOpMode")
public class TeleOpMode extends OpMode {
    private MotorMap motorMap;
    private GamepadWrapper gamepadWrapper;
    private Robot robot;
    private int IntakeConveyorCounter = 0;
    private int WobbleArmCounter = 0;


    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");  //front left = white tape, back left = red tape, front right = blue, back right = black
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        robot = new Robot(hardwareMap, motorMap, "conveyor", "pushy", "intake", "shooter", "wobble");
        gamepadWrapper = new GamepadWrapper();


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

        if (gamepadWrapper.isDown("g1_right_trigger")) {
            robot.SetShooterPower(1);
        } else {
            robot.SetShooterPower(0);
        }

        if (gamepadWrapper.isPressed("g1_b")) {
            robot.SetIntakePower(++IntakeConveyorCounter % 2);
            robot.SetConveyorPower(IntakeConveyorCounter % 2);
        }

        if (gamepadWrapper.isDown("g1_a")) {
            robot.Push();
        } else {
            robot.Retract();
        }

        if (gamepadWrapper.isPressed("g1_x")) {
            ++WobbleArmCounter;
            if (WobbleArmCounter % 2 == 0) {
                robot.Lower();
            } else {
                robot.Raise();
            }
        }

        telemetry.addData("Yaw", robot.GetYaw());
        telemetry.addData("Roll", robot.GetRoll());
        telemetry.addData("Pitch", robot.GetPitch());
    }

    @Override
    public void stop() {

    }
}
