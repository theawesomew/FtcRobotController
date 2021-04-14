package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Vector;

public class XDrive extends DriveBase {
    private DcMotor forwardLeft, forwardRight, backLeft, backRight;
    private double pi = Math.PI;
    private final double ticksPerWeelRotation = 1440;
    private final double wheelDiameter = 4*25.4;
    private final double wheelRotationsPerDegree = (((335*Math.sqrt(2))/(wheelDiameter))/360);
    private final double ticksPerDegree = wheelRotationsPerDegree * ticksPerWeelRotation;
    private final double ticksPerRadian = ticksPerDegree * 180/pi;
    private final double ticksPerMM = ticksPerWeelRotation/(2*pi*wheelDiameter);
    private int counter = 0;

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
        backRight.setPower(power * Math.cos(theta));
        forwardRight.setPower(power * Math.sin(theta));
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

    public int RotateByAngle (double angle, boolean direction, int iterator, Telemetry telemetry) {
        if (!(forwardRight.isBusy() && forwardLeft.isBusy() && backRight.isBusy() && backLeft.isBusy())) {
            forwardLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            forwardRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int targetPosition = (int) Math.round(ticksPerRadian * angle);

            forwardLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            forwardRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            forwardLeft.setTargetPosition(targetPosition);
            forwardRight.setTargetPosition(targetPosition);
            backLeft.setTargetPosition(targetPosition);
            backRight.setTargetPosition(targetPosition);

            float power;

            if (direction) {
                power = -1;
            } else {
                power = 1;
            }

            SetRotation(power);

            return iterator+2;
        }
        return iterator;
    }

    public int StrafeByDistance (double distance, double angle, int iterator, Telemetry telemetry) {
        int forwardLeftDistance = 0, forwardRightDistance = 0, backLeftDistance = 0, backRightDistance = 0;
        if (!forwardRight.isBusy() && !forwardLeft.isBusy() && !backRight.isBusy() && !backLeft.isBusy()) {
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
        } else if ( within(forwardLeft.getCurrentPosition(), forwardLeft.getCurrentPosition()+forwardLeftDistance, 10) &&
                    within(forwardRight.getCurrentPosition(), forwardRight.getCurrentPosition()+forwardRightDistance, 10) &&
                    within(backLeft.getCurrentPosition(), backLeft.getCurrentPosition()+backLeftDistance, 10) &&
                    within(backRight.getCurrentPosition(), backRight.getCurrentPosition()+backRightDistance, 10)
        ) {
            return (iterator + 2);
        }
        return iterator;
    }
}
