package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Vector;

public class XDrive extends DriveBase {
    private DcMotor forwardLeft, forwardRight, backLeft, backRight;
    private double pi = Math.PI;
    private final double ticksPerWheelRotation = 1440;
    private final double wheelDiameter = 4*25.4;
    private final double wheelRotationsPerDegree = (((335*Math.sqrt(2))/(wheelDiameter))/360);
    private final double ticksPerDegree = wheelRotationsPerDegree * ticksPerWheelRotation;
    private final double ticksPerRadian = ticksPerDegree * 180/pi;
    private final double ticksPerMM = ticksPerWheelRotation/(pi*wheelDiameter);
    private int forwardLeftDistance = 0, forwardRightDistance = 0, backLeftDistance = 0, backRightDistance = 0;
    private boolean motorsMoving = false;

    public boolean within (int value, int setValue, int error) {
        if (Math.abs(value-setValue) < error) {
            return true;
        }
        return false;
    }

    public XDrive(MotorMap motorMap) {
        forwardLeft = motorMap.GetMotorMap().get("forwardLeft");
        forwardRight = motorMap.GetMotorMap().get("forwardRight");
        backLeft = motorMap.GetMotorMap().get("backLeft");
        backRight = motorMap.GetMotorMap().get("backRight");
    }

    public void SetStrafe (double power, double angle) {
        double theta = angle - pi/4;
        power = power/Math.abs(power) * Math.min(1, Math.abs(power));
        forwardLeft.setPower(-power * Math.cos(theta));
        forwardRight.setPower(power * Math.sin(theta));
        backRight.setPower(power * Math.cos(theta));
        backLeft.setPower(-power * Math.sin(theta));
    }

    public void SetStrafe (Vector movementVector) throws Exception {
        SetStrafe(movementVector.GetMagnitude(), movementVector.GetAngleBetweenVectors(Vector.X_2));
    }

    public void SetRotation (double power) {
        power = power/Math.abs(power) * Math.min(1, Math.abs(power));
        forwardLeft.setPower(power);
        forwardRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public boolean RotateByAngle (double angle, boolean direction) {
        if (!motorsMoving) {

            int targetPosition = (int) Math.round(ticksPerRadian * angle);

            forwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            forwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forwardLeft.setTargetPosition(forwardLeft.getCurrentPosition() + targetPosition);
            forwardRight.setTargetPosition(forwardRight.getCurrentPosition() + targetPosition);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetPosition);
            backRight.setTargetPosition(backRight.getCurrentPosition() + targetPosition);

            float power;

            if (direction) {
                power = -1;
            } else {
                power = 1;
            }

            SetRotation(power);
            motorsMoving = true;
        } else if (within(forwardLeft.getCurrentPosition(), forwardLeft.getTargetPosition(), 10) &&
                within(forwardRight.getCurrentPosition(), forwardRight.getTargetPosition(), 10) &&
                within(backLeft.getCurrentPosition(), backLeft.getTargetPosition(), 10)             &&
                within(backRight.getCurrentPosition(), backRight.getTargetPosition(), 10)) {
            motorsMoving = false;
            return true;
        }

        return false;
    }

    public boolean StrafeByDistance (double distance, double angle) {
        if (!motorsMoving) {
            double theta = angle - pi / 4;

            forwardLeftDistance = (int) (-Math.round(distance * Math.cos(theta) * ticksPerMM));
            forwardRightDistance = (int) (Math.round(distance * Math.sin(theta) * ticksPerMM));
            backLeftDistance = (int) (-Math.round(distance * Math.sin(theta) * ticksPerMM));
            backRightDistance = (int) (Math.round(distance * Math.cos(theta) * ticksPerMM));

            forwardLeft.setTargetPosition(forwardLeft.getCurrentPosition() + forwardLeftDistance);
            forwardRight.setTargetPosition(forwardRight.getCurrentPosition() + forwardRightDistance);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + backLeftDistance);
            backRight.setTargetPosition(backRight.getCurrentPosition() + backRightDistance);

            forwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            forwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            SetStrafe(1, angle);
            motorsMoving = true;
        } else if ( within(forwardLeft.getCurrentPosition(), forwardLeft.getTargetPosition(), 10) &&
                    within(forwardRight.getCurrentPosition(), forwardRight.getTargetPosition(), 10) &&
                    within(backLeft.getCurrentPosition(), backLeft.getTargetPosition(), 10)             &&
                    within(backRight.getCurrentPosition(), backRight.getTargetPosition(), 10)
           ) {
            motorsMoving = false;
            return true;
        }

        return false;
    }
}
