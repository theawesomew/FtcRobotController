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
    private boolean hasMoved[] = {false, false, false, false};

    @Override
    public void init() {
        motorMap = new MotorMap(hardwareMap, "forwardLeft","forwardRight","backLeft","backRight");
        for (DcMotor motor : motorMap.GetMotorMap().values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(!hasMoved[0]) {
            telemetry.addData("hasMoved1", hasMoved[0]);
            hasMoved[0] = xDrive.StrafeByDistance(1000, Math.PI/2, telemetry);
            telemetry.addData("Move", 1);
        } else if (!hasMoved[1]) {
            telemetry.addData("hasMoved1", hasMoved[1]);
            hasMoved[1] = xDrive.StrafeByDistance(1000, Math.PI, telemetry);
            telemetry.addData("Move", 2);
        } else if (!hasMoved[2]) {
            hasMoved[2] = xDrive.StrafeByDistance(1000, Math.PI/3, telemetry);
        } else if (!hasMoved[3]) {
            hasMoved[3] = xDrive.RotateByAngle(Math.PI, true, telemetry);
        }
    }

    @Override
    public void stop() {

    }
}
