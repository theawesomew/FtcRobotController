package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Mechanisms {

    private DcMotor wobbleMotor;


    public WobbleArm (HardwareMap hardwareMap, String wobbleArmName) {
        wobbleMotor = hardwareMap.dcMotor.get(wobbleArmName);
    }

    public void Lower() {
        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(100);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.1);
        }
    }

    public void Raise() {

    }

    public boolean ExtendPosition() {

        if (!wobbleMotor.isBusy()) {
            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setTargetPosition(100);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleMotor.setPower(0.1);
        }
        return true;
    }

    public boolean RetractPosition() {

        return true;
    }


}
