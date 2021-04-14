package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.MotorMap;
import org.firstinspires.ftc.teamcode.robot.XDrive;

@Autonomous(name="UltimateGoalAutonomousMode", group="Autonomous")
public class AutonomousMode extends OpMode {
    private MotorMap motorMap;
    private XDrive xDrive;
    private double[] driveValues = {1000, Math.PI/2, 1000, Math.PI, 0, 0};
    private int iteration = 0;

    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData(motor.toString(), motor.getCurrentPosition());
        }
        xDrive = new XDrive(motorMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Drive Distance", driveValues[iteration]);
        telemetry.addData("Drive Angle", driveValues[iteration+1]);
        telemetry.addData("Iteration", iteration);
        if (iteration < driveValues.length-1) {
            iteration = xDrive.StrafeByDistance(driveValues[iteration], driveValues[iteration + 1], iteration, telemetry);
        }
    }

    @Override
    public void stop() {

    }
}
