package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.Vector;

import java.util.HashMap;

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
    private HashMap<String, Double> strafePower = new HashMap<String, Double>();
    private HashMap<String, Double> rotationPower = new HashMap<String, Double>();
    private HashMap<String, DcMotor> driveMotors = new HashMap<String, DcMotor>();

    public boolean within (int value, int setValue, int error) {
        if (Math.abs(value-setValue) < error) {
            return true;
        }
        return false;
    }

    public XDrive(MotorMap motorMap) {
        for (String motorName : motorMap.GetMotorMap().keySet()) {
            strafePower.put(motorName, 0.0);
            rotationPower.put(motorName, 0.0);
            driveMotors.put(motorName, motorMap.GetMotorMap().get(motorName));
        }
    }

    public void Drive (Telemetry telemetry) {
        double scale = 0;
        for (String motorName : strafePower.keySet()) {
            scale = Math.max(scale, strafePower.get(motorName)+rotationPower.get(motorName));
        }

        for (String motorName : strafePower.keySet()) {
            driveMotors.get(motorName).setPower((strafePower.get(motorName)+rotationPower.get(motorName))/(Math.max(1, scale)));
        }
    }

    public void SetStrafe (double power, double angle) {
        power = Math.min(1, power);
        strafePower.put("forwardLeft", -power * Math.cos(angle - Math.PI/4));
        strafePower.put("forwardRight", power * Math.sin(angle - Math.PI/4));
        strafePower.put("backRight", power * Math.cos(angle - Math.PI/4));
        strafePower.put("backLeft", -power*Math.sin(angle - Math.PI/4));
    }

    public void SetStrafe (Vector movementVector) {
        SetStrafe(movementVector.GetMagnitude(), movementVector.GetAngleBetweenVectors(Vector.X_2));
    }

    public void SetRotation (double power) {
        power = Math.abs(power) > 0 ? power/Math.abs(power) * Math.min(1, Math.abs(power)) : 0;
        rotationPower.put("forwardLeft", -power);
        rotationPower.put("forwardRight", -power);
        rotationPower.put("backRight", -power);
        rotationPower.put("backLeft", -power);

    }

    public boolean RotateByAngle (double angle, boolean direction, Telemetry telemetry) {
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
            Drive(telemetry);
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

    public boolean StrafeByDistance (double distance, double angle, Telemetry telemetry) {
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
            Drive(telemetry);
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
