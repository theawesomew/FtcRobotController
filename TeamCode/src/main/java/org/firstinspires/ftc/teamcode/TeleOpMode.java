package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Conveyor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.Pushy;
import org.firstinspires.ftc.teamcode.robot.ServoMap;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.XDrive;


@TeleOp(name="UltimateGoalTeleOpMode", group="LinearOpMode")
public class TeleOpMode extends OpMode {
    private MotorMap motorMap;
    private MotorMap mechMap;
    private ServoMap servoMap;
    private XDrive xDrive;
    private Conveyor conveyor;
    private Shooter shoot;
    private Intake intake;
    private Pushy pushy;





    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft", "forwardRight", "backLeft", "backRight");  //front left = white tape, back left = red tape, front right = blue, back right = black
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        mechMap = new MotorMap(hardwareMap, "conveyor", "intake", "shoot");
        for (DcMotor motor : mechMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        servoMap = new ServoMap(hardwareMap, "pushServo");
        for (Servo servo : servoMap.GetServoMap().values()) {
            servo.setPosition(0);
        }

        xDrive = new XDrive(motorMap);
        conveyor = new Conveyor(mechMap);
        shoot = new Shooter(mechMap);
        intake = new Intake(mechMap);
        pushy = new Pushy(servoMap);


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
         * Conveyor
         */
        if (gamepad1.dpad_up) {
            conveyor.SetPower(-0.5);
        }
        else if (gamepad1.dpad_down) {
            conveyor.SetPower(0.5);
        }
        else {
            conveyor.SetPower(0);
        }

        /**
         * shooting
         */
        if (gamepad1.right_trigger > 0.7){
            /*mechMap.GetMotorMap().get("shoot").setPower(1);*/
            shoot.SetPower(1);
        }
        else {
           /* mechMap.GetMotorMap().get("shoot").setPower(0); */
            shoot.SetPower(0);
        }

        /**
         * intake
         */
        if (gamepad1.b) {
            /*mechMap.GetMotorMap().get("intake").setPower(0.25); */
            intake.SetPower(0.5);
        }
        if (gamepad1.a) {
            /*mechMap.GetMotorMap().get("intake").setPower(0); */
            intake.SetPower(0);
        }

        /**
         * pushy thing
         */
        if (gamepad1.y) {
            /*servoMap.GetServoMap().get("setPosition").setPosition(0.1); */
            pushy.SetPosition(0.2);
        }
        if (gamepad1.x) {
            /*servoMap.GetServoMap().get("pushServo").setPosition(0); */
            pushy.SetPosition(0);
        }


    }

    @Override
    public void stop() {

    }
}
