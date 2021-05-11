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
    private HashMap<String, Vector> strafeVectors = new HashMap<String, Vector>();
    private HashMap<String, Vector> rotationVectors = new HashMap<String, Vector>();
    private HashMap<String, DcMotor> driveMotors = new HashMap<String, DcMotor>();

    public boolean within (int value, int setValue, int error) {
        if (Math.abs(value-setValue) < error) {
            return true;
        }
        return false;
    }

    public XDrive(MotorMap motorMap) {
        for (String motorName : motorMap.GetMotorMap().keySet()) {
            strafeVectors.put(motorName, new Vector(0,0));
            rotationVectors.put(motorName, new Vector(0,0));
            driveMotors.put(motorName, motorMap.GetMotorMap().get(motorName));
        }
    }

    public void Drive () {
        double scale = 0;
        for (String motorName : strafeVectors.keySet()) {
            scale = Math.max(scale, strafeVectors.get(motorName).Add(rotationVectors.get(motorName)).GetMagnitude());
        }

        for (String motorName : strafeVectors.keySet()) {
            Vector driveVector = (strafeVectors.get(motorName).Add(rotationVectors.get(motorName))).Scale(Math.max(1, scale));
            driveMotors.get(motorName).setPower(driveVector.GetMagnitude());
        }
    }

    public void SetStrafe (double power, double angle) {
        power = power/Math.abs(power) * Math.min(1, Math.abs(power));
        strafeVectors.put("forwardLeft", new Vector(-power * Math.cos(angle - Math.PI/4), Math.PI/4));
        strafeVectors.put("forwardRight", new Vector(power * Math.sin(angle - Math.PI/4), 3*Math.PI/4));
        strafeVectors.put("backRight", new Vector(power * Math.cos(angle-Math.PI/4), Math.PI/4));
        strafeVectors.put("backLeft", new Vector(-power*Math.sin(angle-Math.PI/4), 3*Math.PI/4));
    }

    public void SetStrafe (Vector movementVector) throws Exception {
        SetStrafe(movementVector.GetMagnitude(), movementVector.GetAngleBetweenVectors(Vector.X_2));
    }

    public void SetRotation (double power) {
        power = power/Math.abs(power) * Math.min(1, Math.abs(power));
        rotationVectors.put("forwardLeft", new Vector(-power, Math.PI/4));
        rotationVectors.put("forwardRight", new Vector(-power, 3*Math.PI/4));
        rotationVectors.put("backRight", new Vector(-power, Math.PI/4));
        rotationVectors.put("backLeft", new Vector(-power, 3*Math.PI/4));

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
            Drive();
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
            Drive();
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
