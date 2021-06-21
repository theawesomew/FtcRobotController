package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Mechanisms {

    private Servo wobbleServoLeft;
    private Servo wobbleServoRight;

    private boolean moving = false;



    public WobbleArm (HardwareMap hardwareMap, String wobbleArmNameLeft, String wobbleArmNameRight) {
        wobbleServoLeft = hardwareMap.servo.get(wobbleArmNameLeft);
        wobbleServoRight = hardwareMap.servo.get(wobbleArmNameRight);
        wobbleServoLeft.setPosition(0.5);
        wobbleServoRight.setPosition(0.5);
    }



    public void Lower() throws InterruptedException {

        /*if (!wobbleS.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(-600);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(-0.1);
        } */

        if (!moving) {
            wobbleServoLeft.setPosition(0);
            wobbleServoRight.setPosition(1);
            moving = true;
            Thread.sleep(2000);
            moving = false;
        }


    }

    public void Raise() throws InterruptedException {
        /*if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(600);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.15);
        } */

        if (!moving) {
            wobbleServoLeft.setPosition(0.7);
            wobbleServoRight.setPosition(0.3);
            moving = true;
            Thread.sleep(2000);
            moving = false;
        }

    }

    /*public boolean LowerPosition() {

        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(-600);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.1);
        }




    }

    public boolean RaisedPosition() {
        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(600);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.1);
        } else if (within(wobbleMotor.getCurrentPosition(), wobbleMotor.getTargetPosition(), 10)) {
            return true;
        }

        return false;

    }
    */



}
