package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.XDrive;


@TeleOp(name="UltimateGoalTeleOpMode", group="LinearOpMode")
public class TeleOpMode extends OpMode {
    private MotorMap motorMap;
    private MotorMap mechMap;
    private XDrive xDrive;
    private Servo pushServo;


    private boolean toggleIntake = false;

    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        mechMap = new MotorMap(hardwareMap, "conveyor", "intake", "shoot");
        for (DcMotor motor : mechMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        xDrive = new XDrive(motorMap);
        pushServo = hardwareMap.servo.get("pushServo");
        pushServo.setPosition(0.0);
    }


    @Override
    public void init_loop() {

    }

    @Override
    public void start () {

    }

    @Override
    public void loop () {
        double angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double power = Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y+gamepad1.left_stick_x*gamepad1.left_stick_x);
        xDrive.SetStrafe(power, angle);

        if (gamepad1.right_bumper) {
            xDrive.SetRotation(-1);
        } else if (gamepad1.left_bumper) {
            xDrive.SetRotation(1);
        }

        /**
         * Conveyor moving
         */
        if (gamepad1.dpad_up) {
            mechMap.GetMotorMap().get("conveyor").setPower(0.5);
        }
        else if (gamepad1.dpad_down) {
            mechMap.GetMotorMap().get("conveyor").setPower(-0.5);
        }
        else {
            mechMap.GetMotorMap().get("conveyor").setPower(0);
        }

        /**
         * shooting
         */
        if (gamepad1.right_trigger > 0.7){
            mechMap.GetMotorMap().get("shoot").setPower(1);
        }
        else {
            mechMap.GetMotorMap().get("shoot").setPower(0);
        }

        /**
         * intake
         */
        if (gamepad1.b) {
            mechMap.GetMotorMap().get("intake").setPower(0.25);
        }
        if (gamepad1.a) {
            mechMap.GetMotorMap().get("intake").setPower(0);

        }

        /**
         * pushy thing
         */
        if (gamepad1.y) {
            pushServo.setPosition(0.1);
        }
        if (gamepad1.x) {
            pushServo.setPosition(0);
        }


    }

    @Override
    public void stop() {

    }
}
