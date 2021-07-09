package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class MotorMap {
    private HashMap<String, DcMotor> motorMap = new HashMap<String, DcMotor>();

    public MotorMap (HardwareMap hardwareMap, String ...mapValues) {
        for (String value : mapValues) {
            motorMap.put(value, hardwareMap.dcMotor.get(value));
        }

        for (DcMotor motor : motorMap.values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public HashMap<String, DcMotor> GetMotorMap () {
        return motorMap;
    }
}
