package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor extends Mechanisms {
    private DcMotor conveyor;
    public Conveyor(HardwareMap hardwareMap, String motorName) {
        conveyor = hardwareMap.dcMotor.get(motorName);
    }

    public void SetPower (double power) {
        conveyor.setPower(power);
    }

    public boolean runConveyor () {
        if (!conveyor.isBusy()) {
            conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            conveyor.setTargetPosition(-10 * 1440);
            conveyor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.SetPower(-1);
        }
        return true;
    }
}
