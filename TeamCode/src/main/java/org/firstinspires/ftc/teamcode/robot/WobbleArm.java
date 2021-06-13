package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleArm extends Mechanisms {

    private DcMotor wobbleMotor;


    public WobbleArm (HardwareMap hardwareMap, String wobbleArmName) {
        wobbleMotor = hardwareMap.dcMotor.get(wobbleArmName);
    }


    

}
