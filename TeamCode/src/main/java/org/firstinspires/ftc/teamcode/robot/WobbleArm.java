package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Mechanisms {

    private Servo wobbleServoLeft;
    private Servo wobbleServoRight;

    private DcMotor wobbleMotor;

    private boolean moving = false;

    public WobbleArm (HardwareMap hardwareMap, String wobbleArmNameLeft, String wobbleArmNameRight, String wobbleArmMotor) {
        wobbleMotor = hardwareMap.dcMotor.get(wobbleArmMotor);
    }

    public void SetPower (double power) {
        this.wobbleMotor.setPower(power);
    }

    public void MotorLower() {
        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(-3800);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(-0.8);
        }

    }

    public void MotorRaise() {
        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(3800);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.7);
        }
    }

    public void toZero () {
        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(0);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (this.GetCurrentWobblePosition() < 0) {
                wobbleMotor.setPower(0.7);
            } else {
                wobbleMotor.setPower(-0.7);
            }
        }
    }

    public int GetCurrentWobblePosition () {
        return wobbleMotor.getCurrentPosition();
    }
}
