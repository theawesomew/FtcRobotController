package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private XDrive xDrive;
    private Conveyor conveyor;
    private Intake intake;
    private Pushy pushy;
    private Shooter shooter;
    private WobbleArm wobbleArm;
    private BNO055IMU imu;
    private Claw claw;
    private Ramp ramp;
    private ColourSensors colorSensor;
    private ElapsedTime robotTime;
    private double prevTime = -1.0;
    private ElapsedTime intakeTime;
    private double prevTimeIntake = -1.0;


    public Robot (HardwareMap hardwareMap, MotorMap driveMap, String conveyorName, String pushyName,
                  String intakeName, String shooterName, String wobbleArmName, String clawLeftName, String clawRightName, String rampName, String colorSensorRightOne, String colorSensorRightFour) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robotTime = new ElapsedTime();
        robotTime.reset();

        this.xDrive = new XDrive(driveMap, imu);
        this.conveyor = new Conveyor(hardwareMap, conveyorName);
        this.intake = new Intake(hardwareMap, intakeName);
        this.pushy = new Pushy(hardwareMap, pushyName);
        this.shooter = new Shooter(hardwareMap, shooterName);
        this.wobbleArm = new WobbleArm(hardwareMap, wobbleArmName);
        this.ramp = new Ramp(hardwareMap, rampName);
        this.claw = new Claw(hardwareMap, clawLeftName, clawRightName);
        this.colorSensor = new ColourSensors(hardwareMap, colorSensorRightOne, colorSensorRightFour);
    }

    public void Drive (Telemetry telemetry) {
        this.xDrive.Drive(telemetry);
    }

    public void SetStrafe (double power, double angle) {
        this.xDrive.SetStrafe(power, angle);
    }

    public boolean StrafeByDistance (double distance, double angle, Telemetry telemetry) {
        return this.xDrive.StrafeByDistance(distance, angle, telemetry);
    }

    public void SetRotation (double power) {
        this.xDrive.SetRotation(power);
    }

    public boolean RotateByAngle (double angle, boolean direction, Telemetry telemetry) {
        return this.xDrive.RotateByAngle(angle, direction, telemetry);
    }

    public boolean RotateByAngleUsingIMU (double angle, boolean direction, Telemetry telemetry) {
        return this.xDrive.RotateByAngleUsingIMU(angle, direction, telemetry);
    }

    public void SetConveyorPower (double power) {
        this.conveyor.SetPower(-power);
    }

    public boolean RunConveyor () {
        return this.conveyor.runConveyor();
    }

    public void SetShooterPower (double power) {
        this.shooter.SetPower(power);
    }

    public double GetShooterPower () {
        return this.shooter.GetPower();
    }

    public void AdjustedShootPower() {
        this.shooter.AdjustedShootPower();
    }

    public boolean Shoot (double rotations) {
        return this.shooter.Shoot(rotations);
    }

    public void SetIntakePower (double power) {
        this.intake.SetPower(power);
    }

    public void Push () {
        this.pushy.Push();
    }

    public void Retract () {
        this.pushy.Retract();
    }

    public boolean PushThenRetract(Telemetry telemetry) throws InterruptedException {
        return this.pushy.PushThenRetract(telemetry);
    }

    public float GetYaw () {
        return imu.getAngularOrientation().firstAngle;
    }

    public float GetRoll () {
        return imu.getAngularOrientation().secondAngle;
    }

    public float GetPitch () {
        return imu.getAngularOrientation().thirdAngle;
    }

    public void ClawOpen() {
        this.claw.ClawOpen();
    }

    public void ClawClose() {
        this.claw.ClawClose();
    }

    public void Extend() {this.ramp.Extend();}

    public void Return() {this.ramp.Retract();}

    public void Raise() throws InterruptedException {
        this.wobbleArm.Raise();
    }

    public void Lower() throws InterruptedException {this.wobbleArm.Lower();}

    //public boolean LoweredPosition() {return this.wobbleArm.LowerPosition();}

   // public boolean RaisedPosition() {return this.wobbleArm.RaisedPosition();}

    public boolean Up () {return this.ramp.Up();}

    public boolean Down () {return this.ramp.Down();}

    public int GetRed() {return this.colorSensor.GetRed();}

    public boolean Sleep (double milliseconds, Telemetry telemetry) {
        if (prevTime == -1.0) {
            prevTime = robotTime.milliseconds();
            return false;
        } else if (robotTime.milliseconds() - prevTime < milliseconds) {
            telemetry.addData("Timer", robotTime.milliseconds());
            telemetry.addData("Previous Time", prevTime);
            this.SetStrafe(0,0);
            this.SetRotation(0);
            return false;
        } else {
            prevTime = -1.0;
            return true;
        }
    }

    public boolean IntakeSleep (double milliseconds, Telemetry telemetry) {
        if (prevTimeIntake == -1.0) {
            prevTimeIntake = robotTime .milliseconds();
            return false;
        } else if (robotTime.milliseconds() - prevTimeIntake < milliseconds) {
            telemetry.addData("Timer", robotTime.milliseconds());
            telemetry.addData("Previous Time", prevTimeIntake);
            return false;
        } else {
            prevTime = -1.0;
            return true;
        }
    }
}

