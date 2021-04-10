package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Vector;

public class XDrive extends DriveBase {
    private DcMotor forwardLeft, forwardRight, backLeft, backRight;
    private double pi = Math.PI;

    public XDrive (MotorMap motorMap) {

    }

    public void SetStrafe (double magnitude, double angle) {
        double theta = angle - pi/4;
        forwardLeft.setPower(magnitude * Math.cos(theta));
        backRight.setPower(magnitude * Math.cos(theta));
        forwardRight.setPower(magnitude * Math.sin(theta));
        backLeft.setPower(magnitude * Math.sin(theta));
    }

    public void SetStrafe (Vector movementVector) throws Exception {
        SetStrafe(movementVector.GetMagnitude(), movementVector.GetAngle());
    }
}
